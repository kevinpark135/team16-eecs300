#include "../../thermal/MLX90640_Thermal_Camera/thermalCamera.hpp"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <vector>

namespace {
std::vector<int> script;
std::size_t position = 0;
int beginError = 0;
int refreshRate = 4;
unsigned long fakeTime = 0;
void readings(std::vector<int> values) { script = values; position = 0; }
}
unsigned long millis() { return fakeTime; }
void delay(unsigned long duration) { fakeTime += duration; }
int MLX90640_DumpEE(std::uint8_t, std::uint16_t *) { return beginError; }
int MLX90640_ExtractParameters(std::uint16_t *, paramsMLX90640 *) { return 0; }
int MLX90640_SetRefreshRate(std::uint8_t, std::uint8_t) { return 0; }
int MLX90640_GetRefreshRate(std::uint8_t) { return refreshRate; }
int MLX90640_TryGetFrameData(std::uint8_t, std::uint16_t *frame) {
  int value = 2;
  if (position < script.size()) { value = script[position++]; }
  if (value == 0 || value == 1) { frame[833] = static_cast<std::uint16_t>(value); }
  return value;
}
float MLX90640_GetTa(std::uint16_t *, const paramsMLX90640 *) { return 24.0f; }
void MLX90640_CalculateTo(std::uint16_t *frame, const paramsMLX90640 *, float, float, float *result) {
  for (int i = frame[833]; i < 768; i += 2) { result[i] = 25.0f; }
}

int main() {
  ThermalCamera camera;
  assert(camera.begin() == 0);
  beginError = -3;
  assert(camera.begin() == -3);
  beginError = 0;
  refreshRate = 5;
  assert(camera.begin() == -2);
  refreshRate = 4;

  readings({2, 0, 2, 0, 1});
  assert(camera.poll(10) == 0);
  assert(camera.poll(20) == 0);
  assert(std::isfinite(camera.temperatures()[0]) && std::isnan(camera.temperatures()[1]));
  assert(camera.poll(30) == 0);
  assert(camera.poll(40) == 0); // Repeated half-image cannot complete a frame.
  assert(camera.poll(50) == 1);
  for (float temperature : camera.temperatures()) { assert(temperature == 25.0f); }

  readings({0, 1, 0});
  assert(camera.poll(100) == 0);
  assert(camera.poll(1100) == -10); // Do not combine halves across a long gap.
  assert(camera.poll(1110) == 0);
  assert(camera.poll(1120) == 1);

  readings({0, -3, 1, 0});
  assert(camera.poll(1200) == 0);
  assert(camera.poll(1210) == -3);
  assert(camera.poll(1220) == 0);
  assert(camera.poll(1230) == 1);

  const std::uint32_t nearWrap = std::numeric_limits<std::uint32_t>::max() - 10;
  readings({0, 1});
  assert(camera.poll(nearWrap) == 0);
  assert(camera.poll(nearWrap + 100) == 1);
  std::cout << "PASS thermal camera: setup errors, paired subpages, gaps, recovery, timer wrap\n";
}
