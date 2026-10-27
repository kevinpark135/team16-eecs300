#pragma once

#include <cstddef>

// One frame holds 24 rows of 32 temperature samples in Celsius.
const std::size_t THERMAL_PIXEL_COUNT = 768;
const std::size_t THERMAL_PAYLOAD_SIZE = THERMAL_PIXEL_COUNT * sizeof(float);
static_assert(sizeof(float) == 4, "The wire protocol requires 32-bit floats");

struct floatPacket {
    // Store only the payload; packetReader handles the AA BB header separately.
    float data[THERMAL_PIXEL_COUNT] = {};
};
