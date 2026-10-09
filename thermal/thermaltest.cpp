#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <limits>
#include <poll.h>
#include <sstream>

#include <SDL.h>

#include "packetReader.hpp"
#include "blobDetector.hpp"

const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 720;
const int CELL_SIZE = 10;

thermal::BlobDetector gBlobDetector; // Defaults are centralized in blobDetector.hpp.

bool init(const char *device);
void cleanup();
void initializeWaitingPattern();
void onReceived(const floatPacket &packet);
void updateDetectionTitle(const thermal::FrameResult &result);
void drawBlobOverlays(const thermal::FrameResult &result);

SDL_Window *gWindow = nullptr;
SDL_Renderer *gRenderer = nullptr;

bool sdlInitialized = false;

packetReader *reader = nullptr;

SDL_Color *gPixels = nullptr;

int serial_port = -1;
uint64_t gRenderedPackets = 0;

bool init(const char *device)
{
    auto fail = [device](const char *operation, const char *reason)
    {
        std::cerr << device << ": " << operation << ": " << reason << '\n';
        cleanup();
        return false;
    };
    serial_port = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_port < 0)
    {
        if (errno == EBUSY)
            return fail("open", "Resource busy; close Arduino Serial Monitor and any other viewer");
        return fail("open", std::strerror(errno));
    }
#ifdef TIOCEXCL
    if (ioctl(serial_port, TIOCEXCL) != 0)
    {
        if (errno == EBUSY)
            return fail("exclusive access", "Resource busy; close Arduino Serial Monitor and any other viewer");
        return fail("TIOCEXCL", std::strerror(errno));
    }
#endif

    termios tty{};
    if (tcgetattr(serial_port, &tty) != 0)
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
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
        return fail("tcsetattr", std::strerror(errno));

    sdlInitialized = true; // SDL_Quit also cleans up a partially failed SDL_Init.
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        return fail("SDL_Init", SDL_GetError());
    }

    if (!SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1"))
    {
        printf("linear texture filtering not enabled");
    }

    gWindow = SDL_CreateWindow("thermaltest", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (gWindow == nullptr)
    {
        return fail("SDL_CreateWindow", SDL_GetError());
    }

    gRenderer = SDL_CreateRenderer(gWindow, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (gRenderer == nullptr)
    {
        return fail("SDL_CreateRenderer", SDL_GetError());
    }

    reader = new packetReader(onReceived);
    gPixels = new SDL_Color[768]();
    initializeWaitingPattern();
    std::cerr << "[viewer] " << device
              << " opened at 115200 baud, raw 8N1; waiting for AA BB + 3072-byte packets\n";
    std::cerr << "[thermal] Keep the scene empty for " << gBlobDetector.config().calibrationFrames
              << " frames; R resets background/results. received_monotonic_ms is host packet\n"
              << "          reception time, NOT sensor capture time. Blob labels are frame-local.\n";
    updateDetectionTitle(gBlobDetector.snapshot());
    return true;
}

void cleanup()
{
    delete reader;
    reader = nullptr;
    delete[] gPixels;
    gPixels = nullptr;
    if (gRenderer)
        SDL_DestroyRenderer(gRenderer);
    if (gWindow)
        SDL_DestroyWindow(gWindow);
    gWindow = nullptr;
    gRenderer = nullptr;
    if (sdlInitialized)
        SDL_Quit();
    sdlInitialized = false;
    if (serial_port >= 0)
        ::close(serial_port);
    serial_port = -1;
}

void initializeWaitingPattern()
{
    for (int row = 0; row < 24; ++row)
    {
        for (int column = 0; column < 32; ++column)
        {
            const bool alternate = ((row / 2) + (column / 2)) % 2 != 0;
            gPixels[row * 32 + column] = alternate
                                                    ? SDL_Color{72, 96, 160, 255}
                                                    : SDL_Color{32, 48, 96, 255};
        }
    }
}

void onReceived(const floatPacket &packet)
{
    // Exactly once per complete packet, using raw Celsius values before coloring.
    const auto &result = gBlobDetector.processFrame(packet.data, thermal::Clock::now());
    const auto receivedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        result.receivedAt->time_since_epoch()).count();
    std::cerr << "[thermal] frame=" << result.frameNumber
              << " received_monotonic_ms=" << receivedMs
              << " status=" << thermal::statusName(result.status)
              << " blobCount=";
    if (const auto count = result.blobCount())
        std::cerr << *count;
    else
        std::cerr << "unavailable";
    std::cerr << " validPixels=" << result.validPixelCount << '/' << result.roiPixelCount
              << " invalidPixels=" << result.invalidPixelCount
              << " excludedRoiPixels=" << result.excludedPixelCount
              << " backgroundUnavailable=" << result.backgroundUnavailablePixelCount;
    if (!result.hasDetection())
        std::cerr << " calibration=" << result.calibrationFramesCollected
                  << '/' << gBlobDetector.config().calibrationFrames;
    if (result.dataDegraded)
        std::cerr << " quality=DEGRADED";
    std::cerr << '\n';
    for (const auto &blob : result.blobs)
    {
        const auto &box = blob.boundingBox;
        std::cerr << std::fixed << std::setprecision(2)
                  << "[blob] frame=" << result.frameNumber << " label=" << blob.label
                  << " centroid=(" << blob.centroidX << ',' << blob.centroidY << ')'
                  << " areaPixels=" << blob.areaPixels
                  << " bbox=(" << box.minX << ',' << box.minY << ")-("
                  << box.maxX << ',' << box.maxY << ')'
                  << " mean=" << blob.meanTemperatureC << "C max=" << blob.maxTemperatureC << "C\n";
    }

    float min = std::numeric_limits<float>::infinity();
    float max = -std::numeric_limits<float>::infinity();
    size_t finiteCount = 0;

    for (size_t i = 0; i < sizeof(packet.data) / sizeof(float); ++i)
    {
        if (!std::isfinite(packet.data[i]))
            continue;
        ++finiteCount;
        min = std::min(min, packet.data[i]);
        max = std::max(max, packet.data[i]);
    }

    for (size_t i = 0; i < sizeof(packet.data) / sizeof(float); ++i)
    {
        if (!std::isfinite(packet.data[i]))
        {
            gPixels[i] = SDL_Color{255, 0, 255, 255};
            continue;
        }

        float temp = max > min ? (packet.data[i] - min) / (max - min) : 0.5f;
        temp = std::max(0.0f, std::min(1.0f, temp));
        SDL_Color out;
        out.a = 0xFF;

        if (temp < 0.25)
        {
            out.r = 0;
            out.g = (int)(255 * (temp / 0.25));
            out.b = 255;
        }
        else if (temp < 0.5)
        {
            out.r = 0;
            out.g = 255;
            out.b = (int)(255 * (1.0 - ((temp - 0.25) / 0.25)));
        }
        else if (temp < 0.75)
        {
            out.r = (int)(255 * ((temp - 0.5) / 0.25));
            out.g = 255;
            out.b = 0;
        }
        else
        {
            out.r = 255;
            out.g = (int)(255 * (1.0 - ((temp - 0.75) / 0.25)));
            out.b = 0;
        }

        gPixels[i] = out;
    }

    ++gRenderedPackets;
    static auto lastLog = std::chrono::steady_clock::time_point{};
    const auto now = std::chrono::steady_clock::now();
    if (gRenderedPackets == 1 || now - lastLog >= std::chrono::seconds(1))
    {
        std::cerr << "[temperature] packet=" << gRenderedPackets
                  << " finite=" << finiteCount << "/768";
        if (finiteCount > 0)
        {
            std::cerr << std::fixed << std::setprecision(2)
                      << " min=" << min << "C max=" << max << "C";
        }
        else
        {
            std::cerr << " no finite temperatures; invalid pixels are magenta";
        }
        if (finiteCount < 768)
            std::cerr << " invalid=" << (768 - finiteCount);
        std::cerr << '\n';
        lastLog = now;
    }
}

void updateDetectionTitle(const thermal::FrameResult &result)
{
    std::ostringstream title;
    title << "Thermal | Blobs: ";
    if (const auto count = result.blobCount())
        title << *count;
    else
        title << "--";
    title << " | " << thermal::statusName(result.status);
    if (result.status == thermal::DetectionStatus::Calibrating)
        title << ' ' << result.calibrationFramesCollected << '/' << gBlobDetector.config().calibrationFrames;
    else if (!result.backgroundReady && result.status != thermal::DetectionStatus::Waiting)
        title << " | CALIBRATING " << result.calibrationFramesCollected
              << '/' << gBlobDetector.config().calibrationFrames;
    if (result.dataDegraded && result.status != thermal::DetectionStatus::Degraded)
        title << " | DEGRADED";
    if (result.receivedAt)
        title << " | excluded ROI: " << result.excludedPixelCount
              << " | invalid: " << result.invalidPixelCount;
    const std::string text = title.str();
    static std::string previousTitle;
    if (text != previousTitle)
    {
        SDL_SetWindowTitle(gWindow, text.c_str());
        previousTitle = text;
    }
}

void drawBlobOverlays(const thermal::FrameResult &result)
{
    if (!result.hasDetection())
        return;
    for (const auto &blob : result.blobs)
    {
        const auto &box = blob.boundingBox;
        SDL_Rect rect{box.minX * CELL_SIZE, box.minY * CELL_SIZE,
                      (box.maxX - box.minX + 1) * CELL_SIZE,
                      (box.maxY - box.minY + 1) * CELL_SIZE};
        SDL_SetRenderDrawColor(gRenderer, 255, 255, 255, 255);
        SDL_RenderDrawRect(gRenderer, &rect);
        // Sensor coordinates name cells; +0.5 puts the cross at the cell center.
        const int cx = static_cast<int>(std::lround((blob.centroidX + 0.5) * CELL_SIZE));
        const int cy = static_cast<int>(std::lround((blob.centroidY + 0.5) * CELL_SIZE));
        const int radius = CELL_SIZE / 2;
        SDL_SetRenderDrawColor(gRenderer, 255, 0, 0, 255);
        SDL_RenderDrawLine(gRenderer, cx - radius, cy, cx + radius, cy);
        SDL_RenderDrawLine(gRenderer, cx, cy - radius, cx, cy + radius);
    }
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <serial-device>\n";
        return 1;
    }
    if (!init(argv[1]))
    {
        return 1;
    }
    // Start the no-packet timeout after serial/SDL initialization has finished.
    gBlobDetector = thermal::BlobDetector(gBlobDetector.config());

    int exitCode = 0;
    bool quit = false;
    SDL_Event e;
    uint64_t readCalls = 0;
    auto lastStatusLog = std::chrono::steady_clock::now() - std::chrono::seconds(2);
    auto previousDetectionStatus = thermal::DetectionStatus::Waiting;

    while (!quit)
    {
        while (SDL_PollEvent(&e) != 0)
        {
            if (e.type == SDL_QUIT)
            {
                quit = true;
            }
            else if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_r && e.key.repeat == 0)
            {
                gBlobDetector.reset();
                initializeWaitingPattern();
                std::cerr << "[thermal] background/results reset; CALIBRATING: keep the scene empty\n";
            }
        }

        if (quit)
            break;

        // Bound the wait so SDL events remain responsive even with no sensor data.
        pollfd port{serial_port, POLLIN, 0};
        int ready = poll(&port, 1, 10);
        if (ready < 0 && errno == EINTR)
            continue;
        if (ready < 0 || (port.revents & (POLLERR | POLLHUP | POLLNVAL)))
        {
            std::cerr << argv[1] << ": serial poll: "
                      << (ready < 0 ? std::strerror(errno) : "device disconnected or unavailable")
                      << '\n';
            exitCode = 1;
            break;
        }
        if (ready > 0 && (port.revents & POLLIN))
        {
            uint8_t read_buffer[4096];
            ssize_t num_bytes = read(serial_port, read_buffer, sizeof(read_buffer));
            ++readCalls;
            if (num_bytes > 0)
                reader->processRaw(read_buffer, static_cast<size_t>(num_bytes));
            else if (num_bytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
            {
                std::cerr << argv[1] << ": serial read: " << std::strerror(errno) << '\n';
                exitCode = 1;
                break;
            }
        }

        const auto now = std::chrono::steady_clock::now();
        const auto detection = gBlobDetector.snapshot(now);
        updateDetectionTitle(detection);
        if (detection.status != previousDetectionStatus)
        {
            std::cerr << "[viewer] detection=" << thermal::statusName(detection.status);
            if (detection.status == thermal::DetectionStatus::Stale)
                std::cerr << "; no complete packet for " << gBlobDetector.config().staleTimeout.count()
                          << "ms; blob count unavailable; heatmap shows last frame";
            std::cerr << '\n';
            previousDetectionStatus = detection.status;
        }
        if (now - lastStatusLog >= std::chrono::seconds(2))
        {
            std::cerr << "[serial] bytes=" << reader->getTotalBytes()
                      << " reads=" << readCalls
                      << " header1=" << reader->getHeader1Candidates()
                      << " headers=" << reader->getHeadersFound()
                      << " packets=" << reader->getPacketsCompleted()
                      << " state=" << reader->getStateName();
            if (reader->getPayloadBytesRead() > 0)
                std::cerr << " payload=" << reader->getPayloadBytesRead() << "/3072";
            if (reader->getPacketsCompleted() == 0)
                std::cerr << " (showing blue waiting pattern)";
            std::cerr << '\n';
            lastStatusLog = now;
        }

        SDL_SetRenderDrawColor(gRenderer, 0xff, 0xff, 0xff, 0xff);
        SDL_RenderClear(gRenderer);
        for (int i = 0; i < 24; ++i)
        {
            for (int j = 0; j < 32; ++j)
            {
                SDL_Color currentColor = gPixels[i * 32 + j];
                SDL_SetRenderDrawColor(gRenderer, currentColor.r, currentColor.g, currentColor.b, 0xff);

                SDL_Rect pixelRect = {j * CELL_SIZE, i * CELL_SIZE, CELL_SIZE, CELL_SIZE};
                SDL_RenderFillRect(gRenderer, &pixelRect);
            }
        }

        drawBlobOverlays(detection);
        SDL_RenderPresent(gRenderer);
    }
    cleanup();
    return exitCode;
}
