#include <cstdint>

union floatPacket
{
    float data[768];
    uint8_t bytes[3072];
};