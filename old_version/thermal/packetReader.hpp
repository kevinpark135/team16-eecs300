#include <cstdint>
#include <cstddef>
#include <functional>
#include "floatPacket.hpp"

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
    uint64_t totalBytes = 0;
    uint64_t header1Candidates = 0;
    uint64_t headersFound = 0;
    uint64_t packetsCompleted = 0;

    const uint8_t HEADER1 = 0xAA;
    const uint8_t HEADER2 = 0xBB;
    const size_t PAYLOAD_SIZE = 3072;

    std::function<void(const floatPacket &packet)> onReceived;

public:
    explicit packetReader(std::function<void(const floatPacket &packet)> callback)
    {
        onReceived = callback;
    }

    void processRaw(const uint8_t *buffer, size_t size)
    {
        totalBytes += size;
        for (size_t i = 0; i < size; ++i)
        {
            uint8_t byte = buffer[i];

            switch (state)
            {
            case WAIT_HEADER1:
                if (byte == HEADER1)
                {
                    ++header1Candidates;
                    state = WAIT_HEADER2;
                }
                break;
            case WAIT_HEADER2:
                if (byte == HEADER2)
                {
                    ++headersFound;
                    bytesRead = 0;
                    state = READ;
                    break;
                }
                if (byte == HEADER1)
                {
                    ++header1Candidates;
                    state = WAIT_HEADER2;
                }
                else
                {
                    state = WAIT_HEADER1;
                }
                break;
            case READ:
                currentPacket.bytes[bytesRead++] = byte;

                if (bytesRead >= PAYLOAD_SIZE)
                {
                    ++packetsCompleted;
                    onReceived(currentPacket);
                    state = WAIT_HEADER1;
                }
                break;
            }
        }
    }

    uint64_t getTotalBytes() const { return totalBytes; }
    uint64_t getHeader1Candidates() const { return header1Candidates; }
    uint64_t getHeadersFound() const { return headersFound; }
    uint64_t getPacketsCompleted() const { return packetsCompleted; }
    size_t getPayloadBytesRead() const { return state == READ ? bytesRead : 0; }

    const char *getStateName() const
    {
        switch (state)
        {
        case WAIT_HEADER1:
            return "waiting-header-1";
        case WAIT_HEADER2:
            return "waiting-header-2";
        case READ:
            return "reading-payload";
        }
        return "unknown";
    }
};
