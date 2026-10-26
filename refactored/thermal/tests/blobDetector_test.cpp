#include "../blobDetector.hpp"
#include "../packetReader.hpp"

#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using namespace thermal;
const Clock::time_point kStart = Clock::time_point{};

void require(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}

bool countIs(const FrameResult &result, std::size_t expected)
{
    std::size_t count = 0;
    return result.getBlobCount(count) && count == expected;
}

void near(double actual, double expected)
{
    require(std::abs(actual - expected) < 1e-6, "Unexpected numeric result");
}

TemperatureFrame frame(float temperature = 20.0f)
{
    TemperatureFrame data;
    data.fill(temperature);
    return data;
}

void rectangle(TemperatureFrame &data, int x, int y, int width, int height, float temperature = 25.0f)
{
    for (int row = y; row < y + height; ++row)
        for (int column = x; column < x + width; ++column)
            data[row * kWidth + column] = temperature;
}

void calibrate(BlobDetector &detector, const TemperatureFrame &background = frame())
{
    for (std::size_t i = 0; i < detector.config().calibrationFrames; ++i)
    {
        const FrameResult &result = detector.processFrame(background, kStart + std::chrono::milliseconds(i));
        require(result.status == DetectionStatus::Calibrating, "Expected calibration status");
        require(!result.hasDetection(), "Calibration must not report measured zero");
        require(result.blobs.empty(), "Calibration must not detect blobs");
    }
    require(detector.latestResult().backgroundReady, "Background was not finalized");
}

void backgroundOnly()
{
    BlobDetector detector({}, kStart);
    require(detector.snapshot(kStart).status == DetectionStatus::Waiting, "Expected initial waiting state");
    require(!detector.latestResult().hasDetection(), "Waiting must not report measured zero");
    calibrate(detector);
    const Clock::time_point now = kStart + std::chrono::milliseconds(100);
    const FrameResult &result = detector.processFrame(frame(), now);
    require(result.status == DetectionStatus::Ready, "Expected ready state");
    require(countIs(result, 0), "Background should have zero blobs");
    require(result.validPixelCount == 768 && result.excludedPixelCount == 0, "Expected all pixels usable");
    require(result.frameNumber == 11 && result.hasReceivedFrame && result.receivedAt == now, "Frame metadata mismatch");
}

void oneBlobGeometryAndTemperatures()
{
    BlobDetector detector;
    calibrate(detector);
    TemperatureFrame data = frame();
    // Nonrectangular component: x/y arithmetic mean must not be bbox midpoint
    // or weighted by temperature. All values are below 37C.
    data[3 * kWidth + 2] = 23.0f;
    data[3 * kWidth + 3] = 24.0f;
    data[4 * kWidth + 2] = 25.0f;
    data[5 * kWidth + 2] = 28.0f;
    const FrameResult &result = detector.processFrame(data);
    require(countIs(result, 1), "Expected one blob");
    const Blob &blob = result.blobs.front();
    require(blob.label == 1 && blob.areaPixels == 4, "Label/area mismatch");
    near(blob.centroidX, 2.25);
    near(blob.centroidY, 3.75);
    require(blob.boundingBox.minX == 2 && blob.boundingBox.minY == 3 &&
            blob.boundingBox.maxX == 3 && blob.boundingBox.maxY == 5, "Bounding box mismatch");
    near(blob.meanTemperatureC, 25.0);
    near(blob.maxTemperatureC, 28.0);
    require(countIs(detector.processFrame(frame()), 0), "Fresh empty frame must immediately clear blobs");
}

void separatedBlobsAndNoise()
{
    BlobDetector detector;
    calibrate(detector);
    TemperatureFrame data = frame();
    rectangle(data, 1, 2, 2, 2);
    rectangle(data, 20, 10, 3, 2, 27.0f);
    rectangle(data, 10, 20, 3, 1); // Area 3, rejected.
    rectangle(data, 25, 20, 2, 2, 22.99f); // Below delta threshold, rejected.
    const FrameResult &result = detector.processFrame(data);
    require(countIs(result, 2), "Expected two accepted blobs, excluding noise");
    require(result.blobs[0].areaPixels == 4 && result.blobs[1].areaPixels == 6, "Unexpected component areas");
    require(result.blobs[0].label == 1 && result.blobs[1].label == 2, "Labels must be frame-local");
    near(result.blobs[0].centroidX, 1.5);
    near(result.blobs[0].centroidY, 2.5);
    near(result.blobs[1].centroidX, 21.0);
    near(result.blobs[1].centroidY, 10.5);
}

void diagonalAndConnectedLimit()
{
    BlobDetector detector;
    calibrate(detector);
    TemperatureFrame diagonal = frame();
    for (int i = 0; i < 4; ++i)
        diagonal[(i + 2) * kWidth + (i + 2)] = 25.0f;
    require(countIs(detector.processFrame(diagonal), 1), "Diagonal neighbors require 8-connectivity");

    TemperatureFrame data = frame();
    rectangle(data, 2, 10, 2, 2);
    rectangle(data, 6, 10, 2, 2);
    require(countIs(detector.processFrame(data), 2), "Separated regions should remain separate");
    rectangle(data, 4, 10, 2, 1); // Warm bridge between regions, no morphology.
    const FrameResult &merged = detector.processFrame(data);
    require(countIs(merged, 1) && merged.blobs[0].areaPixels == 10, "Connected regions must merge");
}

void edgesAndFullFrame()
{
    BlobDetector detector;
    calibrate(detector);
    TemperatureFrame data = frame();
    rectangle(data, 30, 0, 2, 2);
    rectangle(data, 0, 2, 2, 2);
    require(countIs(detector.processFrame(data), 2), "Rows must not wrap across image edges");
    const FrameResult &all = detector.processFrame(frame(23.0f)); // Exact threshold is inclusive.
    require(countIs(all, 1) && all.blobs[0].areaPixels == 768, "Full-frame BFS/threshold failed");
    near(all.blobs[0].centroidX, 15.5);
    near(all.blobs[0].centroidY, 11.5);
    require(all.blobs[0].boundingBox.maxX == 31 && all.blobs[0].boundingBox.maxY == 23, "Full bbox mismatch");
}

void invalidCurrentData()
{
    BlobDetector detector;
    calibrate(detector);
    TemperatureFrame data = frame();
    rectangle(data, 4, 4, 2, 2);
    data[0] = std::numeric_limits<float>::quiet_NaN();
    data[1] = std::numeric_limits<float>::infinity();
    data[2] = -std::numeric_limits<float>::infinity();
    const FrameResult &partial = detector.processFrame(data);
    require(partial.status == DetectionStatus::Degraded && partial.dataDegraded, "Expected degraded status");
    require(countIs(partial, 1) && partial.validPixelCount == 765, "Invalid pixels must be excluded");
    require(partial.invalidPixelCount == 3 && partial.excludedPixelCount == 3, "Exclusion counts mismatch");
    const FrameResult &invalid = detector.processFrame(frame(std::numeric_limits<float>::quiet_NaN()));
    require(invalid.status == DetectionStatus::Invalid && !invalid.hasDetection(), "Invalid is not measured zero");
    require(invalid.validPixelCount == 0 && invalid.blobs.empty(), "Invalid must clear previous evidence");
    require(detector.processFrame(frame()).status == DetectionStatus::Ready, "Valid data should recover");
}

void medianAndInsufficientBackground()
{
    BlobDetector detector;
    const float values[] = {18, 19, 20, 20, 20, 20, 21, 22, 100,
                            std::numeric_limits<float>::infinity()};
    for (int i = 0; i < 10; ++i)
    {
        TemperatureFrame background = frame(values[i]);
        background[0] = i < 4 ? 20.0f : std::numeric_limits<float>::quiet_NaN(); // Below required 5.
        background[1] = i < 5 ? 20.0f : std::numeric_limits<float>::infinity(); // Exactly 5: usable.
        const FrameResult &result = detector.processFrame(background);
        if (i == 0)
            require(result.status == DetectionStatus::Calibrating, "Expected calibration");
        if (i == 4)
            require(result.dataDegraded && result.invalidPixelCount == 1, "Report degraded calibration quality");
    }
    TemperatureFrame data = frame();
    rectangle(data, 5, 5, 2, 2, 23.0f); // Median 20, exact delta 3 (mean would fail).
    data[0] = 100.0f; // Uncalibrated pixel cannot form foreground.
    const FrameResult &result = detector.processFrame(data);
    require(result.status == DetectionStatus::Degraded && countIs(result, 1), "Median/background exclusion failed");
    require(result.validPixelCount == 767 && result.backgroundUnavailablePixelCount == 1 &&
            result.excludedPixelCount == 1 && result.invalidPixelCount == 0, "Insufficient sample accounting failed");
}

void evenSampleMedian()
{
    DetectorConfig config;
    config.calibrationFrames = 4;
    BlobDetector detector(config);
    for (float value : {18.0f, 20.0f, 22.0f, 100.0f})
        detector.processFrame(frame(value));
    // Median is (20+22)/2 = 21, rather than selecting either middle sample.
    require(countIs(detector.processFrame(frame(23.0f)), 0), "Even median too low");
    require(countIs(detector.processFrame(frame(24.0f)), 1), "Even median too high");
}

void invalidBackground()
{
    BlobDetector detector;
    for (int i = 0; i < 10; ++i)
    {
        const FrameResult &result = detector.processFrame(frame(std::numeric_limits<float>::infinity()));
        require(result.status == DetectionStatus::Invalid && !result.hasDetection(), "Invalid calibration frame expected");
    }
    const FrameResult &result = detector.processFrame(frame(25.0f));
    require(result.status == DetectionStatus::Invalid && !result.hasDetection(), "No valid baseline must be invalid");
    require(result.backgroundUnavailablePixelCount == 768 && result.validPixelCount == 0, "Invalid baseline accounting failed");
}

void stationaryBlob()
{
    BlobDetector detector;
    calibrate(detector);
    TemperatureFrame data = frame();
    rectangle(data, 8, 9, 2, 2);
    for (int i = 0; i < 200; ++i)
        require(countIs(detector.processFrame(data), 1), "Stationary blob absorbed into background");
}

void roiAndSettings()
{
    DetectorConfig config;
    config.calibrationFrames = 3;
    config.deltaThresholdC = 5.0f;
    config.minBlobArea = 6;
    config.roi = {4, 6, 3, 2};
    BlobDetector detector(config);
    calibrate(detector);
    TemperatureFrame data = frame();
    rectangle(data, 2, 6, 5, 2, 25.0f); // Only six ROI pixels may enter the blob.
    data[0] = std::numeric_limits<float>::quiet_NaN(); // Report even outside ROI.
    const FrameResult &result = detector.processFrame(data);
    require(countIs(result, 1) && result.blobs[0].areaPixels == 6, "ROI/min area settings failed");
    require(result.status == DetectionStatus::Degraded && result.validPixelCount == 6 &&
            result.roiPixelCount == 6 && result.excludedPixelCount == 0, "ROI quality accounting failed");
    near(result.blobs[0].centroidX, 5.0);
    near(result.blobs[0].centroidY, 6.5);
    require(result.blobs[0].boundingBox.minX == 4 && result.blobs[0].boundingBox.maxX == 6, "ROI bbox mismatch");
    rectangle(data, 4, 6, 3, 2, 24.99f);
    require(countIs(detector.processFrame(data), 0), "Custom threshold ignored");
}

void staleAndRecovery()
{
    DetectorConfig config;
    config.staleTimeout = std::chrono::milliseconds(50);
    BlobDetector detector(config, kStart);
    require(detector.snapshot(kStart + std::chrono::milliseconds(49)).status == DetectionStatus::Waiting, "Premature stale");
    require(detector.snapshot(kStart + std::chrono::milliseconds(50)).status == DetectionStatus::Stale, "Waiting timeout failed");
    calibrate(detector);
    TemperatureFrame data = frame();
    rectangle(data, 5, 5, 2, 2);
    const Clock::time_point receivedAt = kStart + std::chrono::milliseconds(100);
    detector.processFrame(data, receivedAt);
    require(countIs(detector.snapshot(receivedAt + std::chrono::milliseconds(49)), 1), "Fresh count missing");
    const FrameResult stale = detector.snapshot(receivedAt + std::chrono::milliseconds(50));
    require(stale.status == DetectionStatus::Stale && !stale.hasDetection() && stale.blobs.empty(), "Stale evidence must be unavailable");
    require(stale.hasReceivedFrame && stale.receivedAt == receivedAt && stale.frameNumber == 11, "Stale must retain reception metadata");
    require(detector.latestResult().blobs.size() == 1, "Snapshot should not mutate last frame");
    const FrameResult &recovered = detector.processFrame(frame(), receivedAt + std::chrono::milliseconds(200));
    require(recovered.status == DetectionStatus::Ready && countIs(recovered, 0), "Fresh empty frame must recover");
}

void resetAndRecalibration()
{
    BlobDetector detector;
    calibrate(detector);
    TemperatureFrame data = frame();
    rectangle(data, 1, 1, 2, 2);
    require(countIs(detector.processFrame(data), 1), "Test setup failed");
    const Clock::time_point resetAt = kStart + std::chrono::seconds(1);
    detector.reset(resetAt);
    const FrameResult afterReset = detector.snapshot(resetAt);
    require(afterReset.status == DetectionStatus::Calibrating && !afterReset.hasDetection(), "Reset must enter calibration");
    require(afterReset.blobs.empty() && !afterReset.backgroundReady && !afterReset.hasReceivedFrame &&
            afterReset.calibrationFramesCollected == 0, "Reset must clear all previous evidence");
    require(detector.snapshot(resetAt + std::chrono::seconds(2)).status == DetectionStatus::Stale, "Reset timeout failed");
    calibrate(detector, frame(30.0f));
    const FrameResult &result = detector.processFrame(frame(30.0f));
    require(countIs(result, 0) && result.frameNumber == 22, "New baseline or session frame number incorrect");
    require(countIs(detector.processFrame(frame(33.0f)), 1), "New baseline not used");
}

void requireRejected(DetectorConfig config)
{
    try
    {
        BlobDetector detector(config);
    }
    catch (const std::invalid_argument &)
    {
        return;
    }
    throw std::runtime_error("Bad configuration was accepted");
}

void configurationValidation()
{
    DetectorConfig config;
    config.calibrationFrames = 0; requireRejected(config);
    config = {}; config.minBlobArea = 0; requireRejected(config);
    config = {}; config.deltaThresholdC = std::numeric_limits<float>::quiet_NaN(); requireRejected(config);
    config = {}; config.deltaThresholdC = -1.0f; requireRejected(config);
    config = {}; config.roi = {31, 0, 2, 24}; requireRejected(config);
    config = {}; config.roi.width = 0; requireRejected(config);
    config = {}; config.staleTimeout = std::chrono::milliseconds(0); requireRejected(config);
}

void sendBytes(packetReader &reader, BlobDetector &detector,
               const std::uint8_t *bytes, std::size_t size, std::size_t &completed)
{
    for (std::size_t i = 0; i < size; ++i)
    {
        if (reader.processByte(bytes[i]))
        {
            ++completed;
            detector.processFrame(reader.packet().data, kStart + std::chrono::milliseconds(completed));
        }
    }
}

void sendFrame(packetReader &reader, BlobDetector &detector,
               const TemperatureFrame &data, std::size_t &completed)
{
    // Include noise and overlapping header candidates before a valid frame.
    std::vector<std::uint8_t> bytes{0x12, 0xAA, 0xAA, 0xBB};
    const std::size_t offset = bytes.size();
    bytes.resize(offset + THERMAL_PAYLOAD_SIZE);
    std::memcpy(bytes.data() + offset, data.data(), THERMAL_PAYLOAD_SIZE);
    const std::size_t before = completed;
    sendBytes(reader, detector, bytes.data(), 3, completed);
    sendBytes(reader, detector, bytes.data() + 3, bytes.size() - 4, completed);
    require(completed == before, "Partial packet ran detection");
    sendBytes(reader, detector, bytes.data() + bytes.size() - 1, 1, completed);
    require(completed == before + 1, "Complete packet must run detection exactly once");
}

void completePacketIntegration()
{
    BlobDetector detector;
    packetReader reader;
    std::size_t completed = 0;
    for (int i = 0; i < 10; ++i)
        sendFrame(reader, detector, frame(), completed);
    TemperatureFrame data = frame();
    rectangle(data, 10, 10, 2, 2);
    sendFrame(reader, detector, data, completed);
    require(countIs(detector.latestResult(), 1) && detector.latestResult().frameNumber == 11,
            "Packet Celsius data not passed through correctly");
    sendFrame(reader, detector, frame(), completed);
    require(countIs(detector.latestResult(), 0) && reader.getPacketsCompleted() == 12,
            "Complete empty packet did not clear blobs");
}
void packetBytesAndCounters()
{
    floatPacket expected;
    expected.data[0] = -0.0f;
    expected.data[1] = std::numeric_limits<float>::infinity();
    expected.data[2] = std::numeric_limits<float>::quiet_NaN();
    const std::uint8_t embeddedHeader[] = {0xAA, 0xBB, 0x20, 0x40};
    std::memcpy(&expected.data[3], embeddedHeader, sizeof(embeddedHeader));
    std::vector<std::uint8_t> bytes{0x12, 0xAA, 0xAA, 0xBB};
    bytes.resize(4 + THERMAL_PAYLOAD_SIZE);
    std::memcpy(bytes.data() + 4, expected.data, THERMAL_PAYLOAD_SIZE);

    packetReader reader;
    for (std::size_t i = 0; i + 1 < bytes.size(); ++i)
    {
        require(!reader.processByte(bytes[i]), "Packet completed too soon");
    }
    require(reader.getPayloadBytesRead() == THERMAL_PAYLOAD_SIZE - 1, "Partial byte count wrong");
    require(reader.processByte(bytes.back()), "Final byte did not complete packet");
    require(std::memcmp(reader.packet().data, expected.data, THERMAL_PAYLOAD_SIZE) == 0,
            "Packet bits changed during byte-to-float conversion");
    require(reader.getTotalBytes() == bytes.size() && reader.getHeader1Candidates() == 2 &&
            reader.getHeadersFound() == 1 && reader.getPacketsCompleted() == 1 &&
            reader.getPayloadBytesRead() == 0, "Parser counters/state mismatch");

    FrameResult unavailable;
    std::size_t count = 123;
    require(!unavailable.getBlobCount(count) && count == 123, "Unavailable count overwrote output");
}
} // namespace

int main()
{
    struct TestCase
    {
        const char *name;
        void (*run)();
    };
    const TestCase tests[] = {
        {"background only and frame metadata", backgroundOnly},
        {"one blob, centroid/bbox/temperature and immediate zero", oneBlobGeometryAndTemperatures},
        {"two separated blobs and small noise", separatedBlobsAndNoise},
        {"8-connectivity and connected-region limitation", diagonalAndConnectedLimit},
        {"image edges, full-frame BFS and inclusive threshold", edgesAndFullFrame},
        {"NaN/Inf, degraded/invalid status and recovery", invalidCurrentData},
        {"median and insufficient background samples", medianAndInsufficientBackground},
        {"even-sample median", evenSampleMedian},
        {"invalid calibration and invalid background", invalidBackground},
        {"stationary blob is never absorbed", stationaryBlob},
        {"ROI and configurable thresholds/area", roiAndSettings},
        {"stale timeout and fresh-frame recovery", staleAndRecovery},
        {"reset clears results and relearns background", resetAndRecalibration},
        {"configuration validation", configurationValidation},
        {"fragmented packets run detection once per complete frame", completePacketIntegration},
        {"packet byte preservation, parser counters and unavailable output", packetBytesAndCounters}
    };
    const std::size_t testCount = sizeof(tests) / sizeof(tests[0]);
    std::size_t failures = 0;
    for (const TestCase &test : tests)
    {
        try
        {
            test.run();
            std::cout << "PASS " << test.name << '\n';
        }
        catch (const std::exception &error)
        {
            ++failures;
            std::cerr << "FAIL " << test.name << ": " << error.what() << '\n';
        }
    }
    std::cout << testCount - failures << '/' << testCount << " tests passed\n";
    return failures == 0 ? 0 : 1;
}
