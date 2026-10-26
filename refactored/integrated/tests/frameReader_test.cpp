#include "../../thermal/MLX90640_Thermal_Camera/MLX90640_API.h"
#include <cassert>
#include <cstdint>
#include <iostream>
#include <vector>

namespace {
std::vector<std::uint16_t> statuses;
std::size_t statusIndex = 0;
unsigned long fakeTime = 0;
int writeCount = 0;
int frameReads = 0;
int writeError = 0;
int readErrorAddress = -1;
void reset(std::vector<std::uint16_t> values) {
  statuses = values;
  statusIndex = 0;
  fakeTime = 0;
  writeCount = 0;
  frameReads = 0;
  writeError = 0;
  readErrorAddress = -1;
}
}

unsigned long millis() { return fakeTime; }
void delay(unsigned long duration) { fakeTime += duration; }

int MLX90640_I2CRead(std::uint8_t, unsigned int address, unsigned int count, std::uint16_t *data) {
  if (static_cast<int>(address) == readErrorAddress) { return -3; }
  if (address == 0x8000) {
    *data = 0;
    if (statusIndex < statuses.size()) { *data = statuses[statusIndex++]; }
  } else if (address == 0x800D) {
    *data = 0x1234;
  } else {
    assert(address == 0x0400 && count == 832);
    ++frameReads;
    for (unsigned int i = 0; i < count; ++i) { data[i] = static_cast<std::uint16_t>(i); }
  }
  return 0;
}
int MLX90640_I2CWrite(std::uint8_t, unsigned int address, std::uint16_t value) {
  assert(address == 0x8000 && value == 0x0030);
  ++writeCount;
  return writeError;
}

int main() {
  std::uint16_t frame[834] = {};
  reset({0});
  assert(MLX90640_TryGetFrameData(0x33, frame) == 2);
  assert(fakeTime == 0 && writeCount == 0 && frameReads == 0);

  for (std::uint16_t subpage : {0, 1}) {
    reset({8, subpage});
    assert(MLX90640_TryGetFrameData(0x33, frame) == subpage);
    assert(frame[831] == 831 && frame[832] == 0x1234 && frame[833] == subpage);
    assert(fakeTime == 0 && frameReads == 1);
  }
  reset({8, 8, 1});
  assert(MLX90640_TryGetFrameData(0x33, frame) == 1);
  assert(frameReads == 2 && writeCount == 2);

  reset({8, 8, 8, 8, 8, 8});
  assert(MLX90640_TryGetFrameData(0x33, frame) == -8);
  assert(frameReads == 5 && fakeTime == 0);

  for (int address : {0x8000, 0x0400, 0x800D}) {
    reset({8, 0});
    readErrorAddress = address;
    assert(MLX90640_TryGetFrameData(0x33, frame) == -3);
  }
  reset({8});
  writeError = -1;
  assert(MLX90640_TryGetFrameData(0x33, frame) == -1);

  reset({0, 0, 8, 1});
  assert(MLX90640_GetFrameData(0x33, frame) == 1);
  assert(fakeTime == 2);
  reset({});
  assert(MLX90640_GetFrameData(0x33, frame) == -9);
  assert(fakeTime == 2500);
  std::cout << "PASS thermal frame reader: pending, subpages, retries, errors, legacy timeout\n";
}
