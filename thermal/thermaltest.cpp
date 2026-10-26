#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <SDL.h>

#include "packetReader.hpp"
#include "blobDetector.hpp"

const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 720;
const int CELL_SIZE = 10;

class ThermalViewer {
public:
    explicit ThermalViewer(const char *devicePath) : device(devicePath) {}
    ~ThermalViewer();
    // SDL handles and the serial descriptor have exactly one owner.
    ThermalViewer(const ThermalViewer &) = delete;
    ThermalViewer &operator=(const ThermalViewer &) = delete;
    bool initialize();
    int run();

private:
    bool fail(const char *operation, const char *reason) const;
    bool openSerialPort();
    void initializeWaitingPattern();
    bool receiveBytes();
    void onReceived(const floatPacket &packet);
    void logDetection(const thermal::FrameResult &result) const;
    void updateColors(const floatPacket &packet);
    SDL_Color heatmapColor(double temperature) const;
    void updateDetectionTitle(const thermal::FrameResult &result);
    void drawBlobOverlays(const thermal::FrameResult &result) const;
    void drawFrame(const thermal::FrameResult &result) const;
    void logSerialStatus() const;

    // This object owns the serial descriptor and SDL resources.
    const char *device;
    int serialPort = -1;
    SDL_Window *window = nullptr;
    SDL_Renderer *renderer = nullptr;
    bool sdlInitialized = false;
    packetReader reader;
    SDL_Color pixels[THERMAL_PIXEL_COUNT] = {};
    thermal::BlobDetector detector;
    std::uint64_t renderedPackets = 0;
    std::uint64_t readCalls = 0;
    thermal::Clock::time_point lastTemperatureLog{};
    std::string previousTitle;
};

bool ThermalViewer::fail(const char *operation, const char *reason) const {
    std::cerr << device << ": " << operation << ": " << reason << '\n';
    return false;
}

bool ThermalViewer::openSerialPort() {
    // POSIX termios configures the byte stream; these flags disable translation.
    serialPort = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serialPort < 0)
    {
        if (errno == EBUSY)
            return fail("open", "Resource busy; close Arduino Serial Monitor and any other viewer");
        return fail("open", std::strerror(errno));
    }
#ifdef TIOCEXCL
    if (ioctl(serialPort, TIOCEXCL) != 0)
    {
        if (errno == EBUSY)
            return fail("exclusive access", "Resource busy; close Arduino Serial Monitor and any other viewer");
        return fail("TIOCEXCL", std::strerror(errno));
    }
#endif

    termios tty{};
    if (tcgetattr(serialPort, &tty) != 0)
        return fail("tcgetattr", std::strerror(errno));
    cfmakeraw(&tty);
    tty.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB);
    tty.c_cflag |= CS8 | CLOCAL | CREAD;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif
#ifdef CCTS_OFLOW
    tty.c_cflag &= ~CCTS_OFLOW;
#endif
#ifdef CRTS_IFLOW
    tty.c_cflag &= ~CRTS_IFLOW;
#endif
#ifdef CDTR_IFLOW
    tty.c_cflag &= ~CDTR_IFLOW;
#endif
#ifdef CDSR_OFLOW
    tty.c_cflag &= ~CDSR_OFLOW;
#endif
#ifdef CCAR_OFLOW
    tty.c_cflag &= ~CCAR_OFLOW;
#endif
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (cfsetispeed(&tty, B115200) != 0 || cfsetospeed(&tty, B115200) != 0)
        return fail("set baud rate", std::strerror(errno));
    if (tcsetattr(serialPort, TCSANOW, &tty) != 0)
        return fail("tcsetattr", std::strerror(errno));

    return true;
}

bool ThermalViewer::initialize() {
    if (!openSerialPort()) {
        return false;
    }
    sdlInitialized = true;
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        return fail("SDL_Init", SDL_GetError());
    }
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
    window = SDL_CreateWindow("thermaltest", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                              SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (window == nullptr) {
        return fail("SDL_CreateWindow", SDL_GetError());
    }
    renderer = SDL_CreateRenderer(window, -1,
                                  SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (renderer == nullptr) {
        return fail("SDL_CreateRenderer", SDL_GetError());
    }
    initializeWaitingPattern();
    // Start the freshness timeout after potentially slow initialization.
    detector = thermal::BlobDetector(detector.config());
    std::cerr << "[viewer] " << device
              << " opened at 115200 baud, raw 8N1; waiting for AA BB + 3072-byte packets\n";
    std::cerr << "[thermal] Keep the scene empty for " << detector.config().calibrationFrames
              << " frames; R resets background/results. Timestamps are host reception times.\n";
    updateDetectionTitle(detector.snapshot());
    return true;
}

ThermalViewer::~ThermalViewer() {
    // The destructor also handles partially completed initialization.
    if (renderer != nullptr) { SDL_DestroyRenderer(renderer); }
    if (window != nullptr) { SDL_DestroyWindow(window); }
    if (sdlInitialized) { SDL_Quit(); }
    if (serialPort >= 0) { close(serialPort); }
}

void ThermalViewer::initializeWaitingPattern() {
    for (int row = 0; row < thermal::kHeight; ++row) {
        for (int column = 0; column < thermal::kWidth; ++column) {
            const int index = row * thermal::kWidth + column;
            if (((row / 2) + (column / 2)) % 2 != 0) {
                pixels[index] = SDL_Color{72, 96, 160, 255};
            } else {
                pixels[index] = SDL_Color{32, 48, 96, 255};
            }
        }
    }
}

bool ThermalViewer::receiveBytes() {
    std::uint8_t buffer[4096];
    const ssize_t size = read(serialPort, buffer, sizeof(buffer));
    ++readCalls;
    if (size < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            fail("serial read", std::strerror(errno));
            return false;
        }
        return true;
    }
    for (ssize_t i = 0; i < size; ++i) {
        if (reader.processByte(buffer[i])) {
            onReceived(reader.packet());
        }
    }
    return true;
}

void ThermalViewer::onReceived(const floatPacket &packet) {
    // Detection uses Celsius values before any display normalization.
    const thermal::FrameResult &result = detector.processFrame(packet.data);
    logDetection(result);
    updateColors(packet);
}

void ThermalViewer::logDetection(const thermal::FrameResult &result) const {
    const std::chrono::milliseconds receivedMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(result.receivedAt.time_since_epoch());
    std::size_t count = 0;
    std::cerr << "[thermal] frame=" << result.frameNumber
              << " received_monotonic_ms=" << receivedMs.count()
              << " status=" << thermal::statusName(result.status) << " blobCount=";
    if (result.getBlobCount(count)) {
        std::cerr << count;
    } else {
        std::cerr << "unavailable";
    }
    std::cerr << " validPixels=" << result.validPixelCount << '/' << result.roiPixelCount
              << " invalidPixels=" << result.invalidPixelCount
              << " excludedRoiPixels=" << result.excludedPixelCount
              << " backgroundUnavailable=" << result.backgroundUnavailablePixelCount;
    if (!result.hasDetection()) {
        std::cerr << " calibration=" << result.calibrationFramesCollected
                  << '/' << detector.config().calibrationFrames;
    }
    if (result.dataDegraded) { std::cerr << " quality=DEGRADED"; }
    std::cerr << '\n';
    for (const thermal::Blob &blob : result.blobs) {
        const thermal::BoundingBox &box = blob.boundingBox;
        std::cerr << std::fixed << std::setprecision(2)
                  << "[blob] frame=" << result.frameNumber << " label=" << blob.label
                  << " centroid=(" << blob.centroidX << ',' << blob.centroidY << ')'
                  << " areaPixels=" << blob.areaPixels
                  << " bbox=(" << box.minX << ',' << box.minY << ")-("
                  << box.maxX << ',' << box.maxY << ')'
                  << " mean=" << blob.meanTemperatureC << "C max=" << blob.maxTemperatureC << "C\n";
    }
}

SDL_Color ThermalViewer::heatmapColor(double value) const {
    const double temperature = std::max(0.0, std::min(1.0, value));
    SDL_Color color{0, 0, 0, 255};
    if (temperature < 0.25) {
        color.g = static_cast<Uint8>(255 * temperature / 0.25);
        color.b = 255;
    } else if (temperature < 0.5) {
        color.g = 255;
        color.b = static_cast<Uint8>(255 * (1.0 - (temperature - 0.25) / 0.25));
    } else if (temperature < 0.75) {
        color.r = static_cast<Uint8>(255 * (temperature - 0.5) / 0.25);
        color.g = 255;
    } else {
        color.r = 255;
        color.g = static_cast<Uint8>(255 * (1.0 - (temperature - 0.75) / 0.25));
    }
    return color;
}

void ThermalViewer::updateColors(const floatPacket &packet) {
    float minimum = std::numeric_limits<float>::infinity();
    float maximum = -std::numeric_limits<float>::infinity();
    std::size_t finiteCount = 0;
    for (std::size_t i = 0; i < THERMAL_PIXEL_COUNT; ++i) {
        if (std::isfinite(packet.data[i])) {
            minimum = std::min(minimum, packet.data[i]);
            maximum = std::max(maximum, packet.data[i]);
            ++finiteCount;
        }
    }
    for (std::size_t i = 0; i < THERMAL_PIXEL_COUNT; ++i) {
        if (!std::isfinite(packet.data[i])) {
            pixels[i] = SDL_Color{255, 0, 255, 255};
            continue;
        }
        double normalized = 0.5;
        if (maximum > minimum) {
            // Double arithmetic also handles extreme finite float values safely.
            normalized = (static_cast<double>(packet.data[i]) - minimum) /
                         (static_cast<double>(maximum) - minimum);
        }
        pixels[i] = heatmapColor(normalized);
    }
    ++renderedPackets;
    const thermal::Clock::time_point now = thermal::Clock::now();
    if (renderedPackets == 1 || now - lastTemperatureLog >= std::chrono::seconds(1)) {
        std::cerr << "[temperature] packet=" << renderedPackets
                  << " finite=" << finiteCount << '/' << THERMAL_PIXEL_COUNT;
        if (finiteCount > 0) {
            std::cerr << std::fixed << std::setprecision(2)
                      << " min=" << minimum << "C max=" << maximum << "C";
        } else {
            std::cerr << " no finite temperatures; invalid pixels are magenta";
        }
        if (finiteCount < THERMAL_PIXEL_COUNT) {
            std::cerr << " invalid=" << THERMAL_PIXEL_COUNT - finiteCount;
        }
        std::cerr << '\n';
        lastTemperatureLog = now;
    }
}

void ThermalViewer::updateDetectionTitle(const thermal::FrameResult &result) {
    std::ostringstream title;
    std::size_t count = 0;
    title << "Thermal | Blobs: ";
    if (result.getBlobCount(count)) { title << count; }
    else { title << "--"; }
    title << " | " << thermal::statusName(result.status);
    if (result.status == thermal::DetectionStatus::Calibrating) {
        title << ' ' << result.calibrationFramesCollected << '/' << detector.config().calibrationFrames;
    } else if (!result.backgroundReady && result.status != thermal::DetectionStatus::Waiting) {
        title << " | CALIBRATING " << result.calibrationFramesCollected
              << '/' << detector.config().calibrationFrames;
    }
    if (result.dataDegraded && result.status != thermal::DetectionStatus::Degraded) {
        title << " | DEGRADED";
    }
    if (result.hasReceivedFrame) {
        title << " | excluded ROI: " << result.excludedPixelCount
              << " | invalid: " << result.invalidPixelCount;
    }
    if (title.str() != previousTitle) {
        previousTitle = title.str();
        SDL_SetWindowTitle(window, previousTitle.c_str());
    }
}

void ThermalViewer::drawBlobOverlays(const thermal::FrameResult &result) const {
    if (!result.hasDetection()) { return; }
    for (const thermal::Blob &blob : result.blobs) {
        const thermal::BoundingBox &box = blob.boundingBox;
        SDL_Rect rect{box.minX * CELL_SIZE, box.minY * CELL_SIZE,
                      (box.maxX - box.minX + 1) * CELL_SIZE,
                      (box.maxY - box.minY + 1) * CELL_SIZE};
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderDrawRect(renderer, &rect);
        // The extra half-cell places the cross at the pixel center.
        const int x = static_cast<int>(std::lround((blob.centroidX + 0.5) * CELL_SIZE));
        const int y = static_cast<int>(std::lround((blob.centroidY + 0.5) * CELL_SIZE));
        const int radius = CELL_SIZE / 2;
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        SDL_RenderDrawLine(renderer, x - radius, y, x + radius, y);
        SDL_RenderDrawLine(renderer, x, y - radius, x, y + radius);
    }
}

void ThermalViewer::drawFrame(const thermal::FrameResult &result) const {
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);
    for (int row = 0; row < thermal::kHeight; ++row) {
        for (int column = 0; column < thermal::kWidth; ++column) {
            const SDL_Color &color = pixels[row * thermal::kWidth + column];
            SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
            SDL_Rect rect{column * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE};
            SDL_RenderFillRect(renderer, &rect);
        }
    }
    drawBlobOverlays(result);
    SDL_RenderPresent(renderer);
}

void ThermalViewer::logSerialStatus() const {
    std::cerr << "[serial] bytes=" << reader.getTotalBytes() << " reads=" << readCalls
              << " header1=" << reader.getHeader1Candidates()
              << " headers=" << reader.getHeadersFound()
              << " packets=" << reader.getPacketsCompleted()
              << " state=" << reader.getStateName();
    if (reader.getPayloadBytesRead() > 0) {
        std::cerr << " payload=" << reader.getPayloadBytesRead() << "/3072";
    }
    if (reader.getPacketsCompleted() == 0) {
        std::cerr << " (showing blue waiting pattern)";
    }
    std::cerr << '\n';
}

int ThermalViewer::run() {
    thermal::Clock::time_point lastStatusLog = thermal::Clock::now() - std::chrono::seconds(2);
    thermal::DetectionStatus previousStatus = thermal::DetectionStatus::Waiting;
    while (true) {
        SDL_Event event;
        while (SDL_PollEvent(&event) != 0) {
            if (event.type == SDL_QUIT) { return 0; }
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_r && event.key.repeat == 0) {
                detector.reset();
                initializeWaitingPattern();
                std::cerr << "[thermal] background/results reset; CALIBRATING: keep the scene empty\n";
            }
        }
        // poll waits briefly so the window remains responsive without data.
        pollfd port{serialPort, POLLIN, 0};
        const int ready = poll(&port, 1, 10);
        if (ready < 0 && errno == EINTR) { continue; }
        if (ready < 0) {
            fail("serial poll", std::strerror(errno));
            return 1;
        }
        if (port.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            fail("serial poll", "device disconnected or unavailable");
            return 1;
        }
        if (ready > 0 && (port.revents & POLLIN)) {
            if (!receiveBytes()) { return 1; }
        }
        const thermal::Clock::time_point now = thermal::Clock::now();
        const thermal::FrameResult result = detector.snapshot(now);
        updateDetectionTitle(result);
        if (result.status != previousStatus) {
            std::cerr << "[viewer] detection=" << thermal::statusName(result.status);
            if (result.status == thermal::DetectionStatus::Stale) {
                std::cerr << "; no complete packet for " << detector.config().staleTimeout.count()
                          << "ms; blob count unavailable; heatmap shows last frame";
            }
            std::cerr << '\n';
            previousStatus = result.status;
        }
        if (now - lastStatusLog >= std::chrono::seconds(2)) {
            logSerialStatus();
            lastStatusLog = now;
        }
        drawFrame(result);
    }
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <serial-device>\n";
        return 1;
    }
    ThermalViewer viewer(argv[1]);
    if (!viewer.initialize()) { return 1; }
    return viewer.run();
}
