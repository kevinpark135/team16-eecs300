#include <cstdint>
#include <stdio.h>
#include <functional>
#include "floatPacket.hpp"

#define DEBUG true

enum parserState
{
    WAIT_HEADER1,
    WAIT_HEADER2,
    READ
};

class packetReader
{
private:
    parserState state = WAIT_HEADER1;
    size_t bytesRead = 0;
    floatPacket currentPacket;

    const uint8_t HEADER1 = 0xAA;
    const uint8_t HEADER2 = 0xBB;
    const size_t PAYLOAD_SIZE = 3072;

    std::function<void(const floatPacket& packet)> onRecieved;

public:

    packetReader(std::function<void(const floatPacket& packet)> callback)
    {
        onRecieved = callback;
    }

    void processRaw(const uint8_t* buffer, size_t size)
    {
        for (size_t i = 0; i < size; ++i)
        {
            uint8_t byte = buffer[i];

            switch(state)
            {
            case WAIT_HEADER1:
                if (byte == HEADER1) state = WAIT_HEADER2;
                if (DEBUG) std::cout << "header 1 recieved" << std::endl;
                break;
            case WAIT_HEADER2:
                if (byte == HEADER2) 
                {
                    if (DEBUG) std::cout << "header 2 recieved" << std::endl;
                    bytesRead = 0;
                    state = READ;
                    break;
                }
                state = (byte == HEADER1) ? WAIT_HEADER2 : WAIT_HEADER1;
                break;
            case READ:
                currentPacket.bytes[bytesRead++] = byte;

                if (bytesRead >= PAYLOAD_SIZE)
                {
                    onRecieved(currentPacket);
                    state = WAIT_HEADER1;
                }
                break;
            }
        }
    }
};