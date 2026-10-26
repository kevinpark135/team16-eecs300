#pragma once

#include <cstddef>

const std::size_t THERMAL_PIXEL_COUNT = 768;
const std::size_t THERMAL_PAYLOAD_SIZE = THERMAL_PIXEL_COUNT * sizeof(float);
static_assert(sizeof(float) == 4, "The wire protocol requires 32-bit floats");

struct floatPacket {
    float data[THERMAL_PIXEL_COUNT] = {};
};
