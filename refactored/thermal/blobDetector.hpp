#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace thermal
{
inline constexpr int kWidth = 32;
inline constexpr int kHeight = 24;
inline constexpr std::size_t kPixelCount = kWidth * kHeight;
using Clock = std::chrono::steady_clock;
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
    // Host monotonic packet reception time, NOT the sensor capture timestamp.
    // No timestamp exists before the first packet or immediately after reset.
    std::optional<Clock::time_point> receivedAt;
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

    // nullopt distinguishes unavailable evidence from a measured count of zero.
    std::optional<std::size_t> blobCount() const
    {
        if (!hasDetection())
            return std::nullopt;
        return blobs.size();
    }
};

class BlobDetector
{
public:
    explicit BlobDetector(DetectorConfig config = {}, Clock::time_point startedAt = Clock::now())
        : config_(config), awaitingSince_(startedAt)
    {
        const auto &roi = config_.roi;
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
        for (auto &samples : backgroundSamples_)
            samples.clear();
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
        const auto since = result.receivedAt.value_or(awaitingSince_);
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
        const auto &roi = config_.roi;
        return x >= roi.x && x < roi.x + roi.width &&
               y >= roi.y && y < roi.y + roi.height;
    }

    void finishCalibration()
    {
        for (std::size_t i = 0; i < kPixelCount; ++i)
        {
            auto &samples = backgroundSamples_[i];
            if (samples.size() >= config_.minimumBackgroundSamples())
            {
                std::sort(samples.begin(), samples.end());
                const std::size_t middle = samples.size() / 2;
                const double median = samples.size() % 2 != 0
                    ? samples[middle]
                    : (static_cast<double>(samples[middle - 1]) + samples[middle]) / 2.0;
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
        auto &result = lastResult_;
        result.frameNumber = ++framesProcessed_;
        result.receivedAt = receivedAt;
        result.roiPixelCount = static_cast<std::size_t>(config_.roi.width * config_.roi.height);
        for (std::size_t i = 0; i < kPixelCount; ++i)
            result.finitePixelCount += std::isfinite(temperatureC[i]) ? 1 : 0;
        result.invalidPixelCount = kPixelCount - result.finitePixelCount;

        if (!backgroundReady_)
        {
            for (std::size_t i = 0; i < kPixelCount; ++i)
            {
                if (!std::isfinite(temperatureC[i]))
                    continue;
                backgroundSamples_[i].push_back(temperatureC[i]);
                if (insideRoi(static_cast<int>(i % kWidth), static_cast<int>(i / kWidth)))
                    ++result.validPixelCount;
            }
            ++calibrationFramesCollected_;
            if (calibrationFramesCollected_ >= config_.calibrationFrames)
                finishCalibration();
            // Calibration frames never produce a detection, including the last.
            result.status = result.validPixelCount == 0
                ? DetectionStatus::Invalid : DetectionStatus::Calibrating;
        }
        else
        {
            std::array<bool, kPixelCount> foreground{};
            for (std::size_t i = 0; i < kPixelCount; ++i)
            {
                if (!insideRoi(static_cast<int>(i % kWidth), static_cast<int>(i / kWidth)))
                    continue;
                if (!backgroundValid_[i])
                    ++result.backgroundUnavailablePixelCount;
                if (!backgroundValid_[i] || !std::isfinite(temperatureC[i]))
                    continue;
                ++result.validPixelCount;
                foreground[i] = static_cast<double>(temperatureC[i]) - backgroundC_[i]
                                >= config_.deltaThresholdC;
            }
            findComponents(foreground, temperatureC, result.blobs);
            result.status = result.validPixelCount == 0 ? DetectionStatus::Invalid
                : (result.invalidPixelCount != 0 || result.validPixelCount < result.roiPixelCount)
                    ? DetectionStatus::Degraded : DetectionStatus::Ready;
        }

        result.calibrationFramesCollected = calibrationFramesCollected_;
        result.backgroundReady = backgroundReady_;
        result.excludedPixelCount = result.roiPixelCount - result.validPixelCount;
        result.dataDegraded = result.invalidPixelCount != 0 || result.excludedPixelCount != 0;
        return result;
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

            while (head < tail)
            {
                const auto index = queue[head++];
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
                        const auto neighbor = static_cast<std::size_t>(ny * kWidth + nx);
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
