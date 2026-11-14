#include "thermalCamera.hpp"
#include "MLX90640_I2C_Driver.h"
#include <Arduino.h>
#include <cmath>

namespace {
const std::uint8_t SENSOR_ADDRESS = 0x33;
const std::uint8_t REFRESH_RATE = 0x04;
const float EMISSIVITY = 0.95f;
const float AMBIENT_SHIFT_C = 8.0f;
const std::uint32_t SUBPAGE_TIMEOUT_MS = 1000;
}

int ThermalCamera::begin() {
  // The shared bus has already been configured by the integrated sketch.
  delay(100);
  std::uint16_t eeprom[832];
  int status = MLX90640_DumpEE(SENSOR_ADDRESS, eeprom);
  if (status != 0) { return status; }
  status = MLX90640_ExtractParameters(eeprom, &calibration);
  if (status != 0) { return status; }
  status = MLX90640_SetRefreshRate(SENSOR_ADDRESS, REFRESH_RATE);
  if (status != 0) { return status; }
  const int actualRate = MLX90640_GetRefreshRate(SENSOR_ADDRESS);
  if (actualRate < 0) { return actualRate; }
  if (actualRate != REFRESH_RATE) { return -2; }
  return 0;
}

int ThermalCamera::poll(std::uint32_t now) {
  if (collecting && now - startedAt >= SUBPAGE_TIMEOUT_MS) {
    collecting = false;
    received[0] = false;
    received[1] = false;
    return -10;
  }
  std::uint16_t frame[834];
  const int subpage = MLX90640_TryGetFrameData(SENSOR_ADDRESS, frame);
  if (subpage == 2) { return 0; }
  if (subpage < 0) {
    collecting = false;
    received[0] = false;
    received[1] = false;
    return subpage;
  }
  if (!collecting) {
    collecting = true;
    startedAt = now;
    received[0] = false;
    received[1] = false;
    // Missing or uncomputed pixels stay invalid, never become artificial cold data.
    for (int i = 0; i < 768; ++i) { temperatureC[i] = NAN; }
  }
  const float ambient = MLX90640_GetTa(frame, &calibration);
  MLX90640_CalculateTo(frame, &calibration, EMISSIVITY,
                       ambient - AMBIENT_SHIFT_C, temperatureC);
  received[subpage] = true;
  if (!received[0] || !received[1]) { return 0; }
  collecting = false;
  return 1;
}
