#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <algorithm>

#include <SDL2/SDL.h>

#include "packetReader.hpp"

const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 720;

bool init();
void close();
void onRecieved(floatPacket packet);

SDL_Window* gWindow = nullptr;
SDL_Renderer* gRenderer = nullptr;

struct termios *tty = nullptr; 

packetReader *reader = nullptr;

SDL_Color *gPixels = nullptr;

int serial_port = -1;

bool init()
{
    serial_port = open("/dev/ttyUSB0", O_RDWR);
    if (serial_port < 0) {return 1;}

    tty = new termios();

    if (tcgetattr(serial_port, tty) != 0) {return 1;}

    cfsetispeed(tty, B115200);
    cfsetospeed(tty, B115200);

    tcsetattr(serial_port, TCSANOW, tty);

    reader = new packetReader(onRecieved);
    gPixels = new SDL_Color[768]();

    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        printf("window could not be created. %s\n", SDL_GetError());
        return false;
    }

    if (!SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1"))
    {
        printf("linear texture filtering not enabled");
    }

    gWindow = SDL_CreateWindow("thermaltest", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (gWindow == nullptr)
    {
        printf("window could not be created. %s\n", SDL_GetError());
        return false;
    }

    gRenderer = SDL_CreateRenderer(gWindow, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (gRenderer == nullptr)
    {
        printf("renderer could not be created. %s\n", SDL_GetError());
        return false;
    }

    SDL_SetRenderDrawColor(gRenderer, 0xff, 0xff, 0xff, 0xff);

    return true;
}

void close()
{
    SDL_DestroyRenderer(gRenderer);
    SDL_DestroyWindow(gWindow);
    gWindow = nullptr;
    gRenderer = nullptr;

    close(serial_port);
}

void onRecieved(floatPacket packet)
{
    float min = packet.data[0];
    float max = packet.data[0];

    for (size_t i = 1; i < sizeof(packet.data)/sizeof(float); ++i)
    {
        if (packet.data[i] < min) min = packet.data[i];
        if (packet.data[i] > max) max = packet.data[i];
    }

    for (size_t i = 0; i < sizeof(packet.data)/sizeof(float); ++i)
    {
        float temp = (packet.data[i] - min) / (max - min);
        SDL_Color out;
        out.a = 0xFF;

        if (temp < 0.25)
        {
            out.r = 0;
            out.g = (int)(255 * (temp/0.25));
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

int main()
{
    if (!init())
    {
        return 1;
    }

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

        uint8_t read_buffer[4096];
        int num_bytes = read(serial_port, &read_buffer, sizeof(read_buffer));
        if (num_bytes > 0) reader->processRaw(read_buffer, num_bytes);
        if (DEBUG)
        {
            std::cout << num_bytes << std::endl;
        }

        SDL_SetRenderDrawColor(gRenderer, 0xff, 0xff, 0xff, 0xff);
        SDL_RenderClear(gRenderer);
        for (int i = 0; i < 24; ++i)
        {
            for (int j = 0; j < 32; ++j)
            {
                int length = 10;
                
                SDL_Color currentColor = gPixels[i*32 + j];
                SDL_SetRenderDrawColor(gRenderer, currentColor.r, currentColor.g, currentColor.b, 0xff);

                SDL_Rect pixelRect = {i*length, j*length, length, length};
                SDL_RenderFillRect(gRenderer, &pixelRect);
            }
        }

        SDL_RenderPresent(gRenderer);
    }
}