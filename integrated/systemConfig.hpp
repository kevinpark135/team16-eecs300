#pragma once

#include "directionSettings.hpp"
#include "passageCounter.hpp"

namespace settings {
inline passage::CounterConfig passageSettings() {
  passage::CounterConfig config;
  config.windowMs = 2000;
  config.thermalFreshMs = 1500;
  config.lidarFreshMs = 500;
  config.clearMs = 300;
  config.stableThermalFrames = 2;
  config.initialOccupancy = 0;
  return config;
}
const unsigned long I2C_FREQUENCY_HZ = 400000;
const std::uint32_t THERMAL_POLL_MS = 10;
const std::uint32_t STATUS_LOG_MS = 1000;
}
