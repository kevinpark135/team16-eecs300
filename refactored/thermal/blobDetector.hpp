#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace thermal
{
const int kWidth = 32;
const int kHeight = 24;
const std::size_t kPixelCount = kWidth * kHeight;
using Clock = std::chrono::steady_clock;
// std::array is a fixed-size array with size(), fill(), and data().
using TemperatureFrame = std::array<float, kPixelCount>;

struct Roi
{
    int x = 0;
    int y = 0;
    int width = kWidth;
    int height = kHeight;
};

// Experiment starting points, not constants that guarantee human detection.
// Change the viewer's settings here; no SDL/serial dependency is required.
struct DetectorConfig
{
    std::size_t calibrationFrames = 10;
    float deltaThresholdC = 3.0f;
    std::size_t minBlobArea = 4;
    Roi roi{};
    std::chrono::milliseconds staleTimeout{2000};

    // At least half the calibration frames (rounded up) must be finite.
    std::size_t minimumBackgroundSamples() const
    {
        return calibrationFrames / 2 + calibrationFrames % 2;
    }
};

enum class DetectionStatus
{
    Waiting,
    Calibrating,
    Ready,
    Degraded,
    Invalid,
    Stale
};

inline const char *statusName(DetectionStatus status)
{
    switch (status)
    {
    case DetectionStatus::Waiting: return "WAITING";
    case DetectionStatus::Calibrating: return "CALIBRATING";
    case DetectionStatus::Ready: return "READY";
    case DetectionStatus::Degraded: return "DEGRADED";
    case DetectionStatus::Invalid: return "INVALID";
    case DetectionStatus::Stale: return "STALE";
    }
    return "INVALID";
}

struct BoundingBox
{
    // Inclusive sensor coordinates. Index = row * 32 + column.
    int minX = 0;
    int minY = 0;
    int maxX = 0;
    int maxY = 0;
};

struct Blob
{
    // Local to this frame, never a persistent person ID.
    std::size_t label = 0;
    std::size_t areaPixels = 0;
    double centroidX = 0.0; // x = column, arithmetic mean of pixel coordinates
    double centroidY = 0.0; // y = row
    BoundingBox boundingBox{};
    double meanTemperatureC = 0.0;
    float maxTemperatureC = 0.0f;
};

struct FrameResult
{
    std::uint64_t frameNumber = 0; // Complete packets processed; survives reset.
    // Caller-supplied monotonic observation time, NOT the sensor capture timestamp.
    // No timestamp exists before the first packet or immediately after reset.
    bool hasReceivedFrame = false;
    Clock::time_point receivedAt{};
    DetectionStatus status = DetectionStatus::Waiting;
    std::size_t calibrationFramesCollected = 0;
    bool backgroundReady = false;
    // Also reports partial data loss while status is CALIBRATING or STALE.
    bool dataDegraded = false;
    std::size_t roiPixelCount = 0;
    // Finite ROI samples during calibration; usable current/background pairs
    // during detection. Metadata in a STALE snapshot describes the last frame.
    std::size_t validPixelCount = 0;
    std::size_t finitePixelCount = 0; // Entire 768-pixel current frame
    std::size_t invalidPixelCount = 0; // Entire frame: NaN/Inf
    std::size_t backgroundUnavailablePixelCount = 0; // ROI, after calibration
    std::size_t excludedPixelCount = 0; // ROI union of unavailable input pixels
    std::vector<Blob> blobs;

    bool hasDetection() const
    {
        return status == DetectionStatus::Ready || status == DetectionStatus::Degraded;
    }

    // MODIFIES: count only when a current detection is available.
    // EFFECTS: Returns false for waiting, calibration, invalid, or stale data.
    bool getBlobCount(std::size_t &count) const
    {
        if (!hasDetection())
        {
            return false;
        }
        count = blobs.size();
        return true;
    }
};

class BlobDetector
{
public:
    explicit BlobDetector(DetectorConfig config = {}, Clock::time_point startedAt = Clock::now())
        : config_(config), awaitingSince_(startedAt)
    {
        const Roi &roi = config_.roi;
        if (config_.calibrationFrames == 0 || config_.minBlobArea == 0 ||
            !std::isfinite(config_.deltaThresholdC) || config_.deltaThresholdC < 0.0f ||
            config_.staleTimeout.count() <= 0 ||
            roi.x < 0 || roi.x >= kWidth || roi.y < 0 || roi.y >= kHeight ||
            roi.width <= 0 || roi.width > kWidth - roi.x ||
            roi.height <= 0 || roi.height > kHeight - roi.y)
        {
            throw std::invalid_argument("Invalid thermal blob detector configuration");
        }
    }

    const DetectorConfig &config() const { return config_; }
    const FrameResult &latestResult() const { return lastResult_; }

    // MODIFIES: Calibration state and the latest frame result.
    // EFFECTS: Processes Celsius values once, independently of display colors.
    const FrameResult &processFrame(const float (&temperatureC)[kPixelCount],
                                    Clock::time_point receivedAt = Clock::now())
    {
        return processPixels(temperatureC, receivedAt);
    }

    const FrameResult &processFrame(const TemperatureFrame &temperatureC,
                                    Clock::time_point receivedAt = Clock::now())
    {
        return processPixels(temperatureC.data(), receivedAt);
    }

    void reset(Clock::time_point resetAt = Clock::now())
    {
        for (std::size_t i = 0; i < kPixelCount; ++i)
        {
            backgroundSamples_[i].clear();
        }
        backgroundC_.fill(0.0f);
        backgroundValid_.fill(false);
        calibrationFramesCollected_ = 0;
        backgroundReady_ = false;
        awaitingSince_ = resetAt;
        lastResult_ = FrameResult{};
        lastResult_.frameNumber = framesProcessed_;
        lastResult_.status = DetectionStatus::Calibrating;
    }

    // Freshness checks do not run the detector or modify the frozen background.
    FrameResult snapshot(Clock::time_point now = Clock::now()) const
    {
        FrameResult result = lastResult_;
        Clock::time_point since = awaitingSince_;
        if (result.hasReceivedFrame)
        {
            since = result.receivedAt;
        }
        if (now - since >= config_.staleTimeout)
        {
            result.status = DetectionStatus::Stale;
            result.blobs.clear(); // Never expose old blobs as current evidence.
        }
        return result;
    }

private:
    bool insideRoi(int x, int y) const
    {
        const Roi &roi = config_.roi;
        return x >= roi.x && x < roi.x + roi.width &&
               y >= roi.y && y < roi.y + roi.height;
    }

    void finishCalibration()
    {
        for (std::size_t i = 0; i < kPixelCount; ++i)
        {
            std::vector<float> &samples = backgroundSamples_[i];
            if (samples.size() >= config_.minimumBackgroundSamples())
            {
                std::sort(samples.begin(), samples.end());
                const std::size_t middle = samples.size() / 2;
                double median = samples[middle];
                if (samples.size() % 2 == 0)
                {
                    median = (static_cast<double>(samples[middle - 1]) + samples[middle]) / 2.0;
                }
                backgroundC_[i] = static_cast<float>(median);
                backgroundValid_[i] = true;
            }
            samples.clear();
        }
        backgroundReady_ = true;
    }

    const FrameResult &processPixels(const float *temperatureC, Clock::time_point receivedAt)
    {
        lastResult_ = FrameResult{}; // A new empty frame immediately replaces old blobs.
        FrameResult &result = lastResult_;
        result.frameNumber = ++framesProcessed_;
        result.receivedAt = receivedAt;
        result.hasReceivedFrame = true;
        result.roiPixelCount = static_cast<std::size_t>(config_.roi.width * config_.roi.height);
        for (std::size_t i = 0; i < kPixelCount; ++i)
        {
            if (std::isfinite(temperatureC[i]))
            {
                ++result.finitePixelCount;
            }
        }
        result.invalidPixelCount = kPixelCount - result.finitePixelCount;

        if (!backgroundReady_)
        {
            collectBackground(temperatureC, result);
        }
        else
        {
            detectForeground(temperatureC, result);
        }

        result.calibrationFramesCollected = calibrationFramesCollected_;
        result.backgroundReady = backgroundReady_;
        result.excludedPixelCount = result.roiPixelCount - result.validPixelCount;
        result.dataDegraded = result.invalidPixelCount != 0 || result.excludedPixelCount != 0;
        return result;
    }

    void collectBackground(const float *temperatureC, FrameResult &result)
    {
        for (std::size_t i = 0; i < kPixelCount; ++i)
        {
            if (!std::isfinite(temperatureC[i]))
            {
                continue;
            }
            backgroundSamples_[i].push_back(temperatureC[i]);
            if (insideRoi(static_cast<int>(i % kWidth), static_cast<int>(i / kWidth)))
            {
                ++result.validPixelCount;
            }
        }
        ++calibrationFramesCollected_;
        if (calibrationFramesCollected_ >= config_.calibrationFrames)
        {
            finishCalibration();
        }
        // The final calibration frame still does not produce detections.
        result.status = DetectionStatus::Calibrating;
        if (result.validPixelCount == 0)
        {
            result.status = DetectionStatus::Invalid;
        }
    }

    void detectForeground(const float *temperatureC, FrameResult &result) const
    {
        std::array<bool, kPixelCount> foreground{};
        for (std::size_t i = 0; i < kPixelCount; ++i)
        {
            if (!insideRoi(static_cast<int>(i % kWidth), static_cast<int>(i / kWidth)))
            {
                continue;
            }
            if (!backgroundValid_[i])
            {
                ++result.backgroundUnavailablePixelCount;
            }
            if (!backgroundValid_[i] || !std::isfinite(temperatureC[i]))
            {
                continue;
            }
            ++result.validPixelCount;
            foreground[i] = static_cast<double>(temperatureC[i]) - backgroundC_[i]
                            >= config_.deltaThresholdC;
        }
        findComponents(foreground, temperatureC, result.blobs);
        if (result.validPixelCount == 0)
        {
            result.status = DetectionStatus::Invalid;
        }
        else if (result.invalidPixelCount != 0 || result.validPixelCount < result.roiPixelCount)
        {
            result.status = DetectionStatus::Degraded;
        }
        else
        {
            result.status = DetectionStatus::Ready;
        }
    }

    void findComponents(std::array<bool, kPixelCount> &foreground,
                        const float *temperatureC, std::vector<Blob> &blobs) const
    {
        std::array<std::size_t, kPixelCount> queue{};
        for (std::size_t seed = 0; seed < kPixelCount; ++seed)
        {
            if (!foreground[seed])
                continue;
            std::size_t head = 0;
            std::size_t tail = 0;
            queue[tail++] = seed;
            foreground[seed] = false;
            Blob blob;
            const int seedX = static_cast<int>(seed % kWidth);
            const int seedY = static_cast<int>(seed / kWidth);
            blob.boundingBox = {seedX, seedY, seedX, seedY};
            blob.maxTemperatureC = -std::numeric_limits<float>::infinity();
            double sumX = 0.0;
            double sumY = 0.0;
            double sumTemperature = 0.0;

            // Breadth-first search groups all eight touching neighbors.
            while (head < tail)
            {
                const std::size_t index = queue[head++];
                const int x = static_cast<int>(index % kWidth);
                const int y = static_cast<int>(index / kWidth);
                ++blob.areaPixels;
                sumX += x;
                sumY += y;
                sumTemperature += temperatureC[index];
                blob.maxTemperatureC = std::max(blob.maxTemperatureC, temperatureC[index]);
                blob.boundingBox.minX = std::min(blob.boundingBox.minX, x);
                blob.boundingBox.minY = std::min(blob.boundingBox.minY, y);
                blob.boundingBox.maxX = std::max(blob.boundingBox.maxX, x);
                blob.boundingBox.maxY = std::max(blob.boundingBox.maxY, y);

                for (int dy = -1; dy <= 1; ++dy)
                {
                    for (int dx = -1; dx <= 1; ++dx)
                    {
                        if (dx == 0 && dy == 0)
                            continue;
                        const int nx = x + dx;
                        const int ny = y + dy;
                        if (nx < 0 || nx >= kWidth || ny < 0 || ny >= kHeight)
                            continue;
                        const std::size_t neighbor = static_cast<std::size_t>(ny * kWidth + nx);
                        if (foreground[neighbor])
                        {
                            foreground[neighbor] = false; // Visit once, on enqueue.
                            queue[tail++] = neighbor;
                        }
                    }
                }
            }

            if (blob.areaPixels < config_.minBlobArea)
                continue;
            blob.label = blobs.size() + 1;
            blob.centroidX = sumX / blob.areaPixels;
            blob.centroidY = sumY / blob.areaPixels;
            blob.meanTemperatureC = sumTemperature / blob.areaPixels;
            blobs.push_back(blob);
        }
    }

    DetectorConfig config_;
    Clock::time_point awaitingSince_;
    std::uint64_t framesProcessed_ = 0;
    std::size_t calibrationFramesCollected_ = 0;
    bool backgroundReady_ = false;
    std::array<std::vector<float>, kPixelCount> backgroundSamples_;
    TemperatureFrame backgroundC_{};
    std::array<bool, kPixelCount> backgroundValid_{};
    FrameResult lastResult_{};
};
} // namespace thermal
