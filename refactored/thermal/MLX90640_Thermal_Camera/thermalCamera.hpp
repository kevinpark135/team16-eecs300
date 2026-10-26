#pragma once

#include <cstdint>
#include "MLX90640_API.h"

class ThermalCamera {
 public:
  // REQUIRES: Wire is already running; this adapter never restarts the shared bus.
  int begin();
  // EFFECTS: Returns 1 for a complete frame, 0 if pending, or a negative error.
  int poll(std::uint32_t now);
  const float (&temperatures() const)[768] { return temperatureC; }

 private:
  paramsMLX90640 calibration{};
  float temperatureC[768] = {};
  bool received[2] = {false, false};
  bool collecting = false;
  std::uint32_t startedAt = 0;
};
