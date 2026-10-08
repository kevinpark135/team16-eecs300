#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <poll.h>

#include <SDL.h>

#include "packetReader.hpp"

const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 720;

bool init(const char *device);
void cleanup();
void onRecieved(floatPacket packet);

SDL_Window *gWindow = nullptr;
SDL_Renderer *gRenderer = nullptr;

bool sdlInitialized = false;

packetReader *reader = nullptr;

SDL_Color *gPixels = nullptr;

int serial_port = -1;

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
        return fail("open", std::strerror(errno));

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

    reader = new packetReader(onRecieved);
    gPixels = new SDL_Color[768]();
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

void onRecieved(floatPacket packet)
{
    float min = packet.data[0];
    float max = packet.data[0];

    for (size_t i = 1; i < sizeof(packet.data) / sizeof(float); ++i)
    {
        if (packet.data[i] < min)
            min = packet.data[i];
        if (packet.data[i] > max)
            max = packet.data[i];
    }

    for (size_t i = 0; i < sizeof(packet.data) / sizeof(float); ++i)
    {
        float temp = max > min ? (packet.data[i] - min) / (max - min) : 0.0f;
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

    int exitCode = 0;
    bool quit = false;
    SDL_Event e;

    while (!quit)
    {
        while (SDL_PollEvent(&e) != 0)
        {
            if (e.type == SDL_QUIT)
            {
                quit = true;
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
            if (num_bytes > 0)
                reader->processRaw(read_buffer, static_cast<size_t>(num_bytes));
            else if (num_bytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
            {
                std::cerr << argv[1] << ": serial read: " << std::strerror(errno) << '\n';
                exitCode = 1;
                break;
            }
        }

        SDL_SetRenderDrawColor(gRenderer, 0xff, 0xff, 0xff, 0xff);
        SDL_RenderClear(gRenderer);
        for (int i = 0; i < 24; ++i)
        {
            for (int j = 0; j < 32; ++j)
            {
                int length = 10;

                SDL_Color currentColor = gPixels[i * 32 + j];
                SDL_SetRenderDrawColor(gRenderer, currentColor.r, currentColor.g, currentColor.b, 0xff);

                SDL_Rect pixelRect = {j * length, i * length, length, length};
                SDL_RenderFillRect(gRenderer, &pixelRect);
            }
        }

        SDL_RenderPresent(gRenderer);
    }
    cleanup();
    return exitCode;
}
