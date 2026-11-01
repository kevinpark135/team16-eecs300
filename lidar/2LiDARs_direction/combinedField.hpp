#pragma once

#include "multiObjectCounter.hpp"
#include <cstdint>

namespace lidar {
const int FIELD_WIDTH = 16;
const int FIELD_HEIGHT = 8;
const int FIELD_CELLS = FIELD_WIDTH * FIELD_HEIGHT;
const std::uint32_t PAIR_MAX_AGE_MS = 100;

// Map the two rotated 8x8 views into the same coordinates used for detection.
inline int rawZone(int row, int column, bool leftSensor) {
  const int sensorRow = leftSensor ? column : 7 - column;
  const int sensorColumn = leftSensor ? 7 - row : row;
  return sensorRow * 8 + sensorColumn;
}

inline MultiObjectConfig fieldSettings() {
  MultiObjectConfig config;
  // Preserve the old mounting directions: down the displayed rows is IN.
  config.direction.axis = direction_counter::ROWS;
  return config;
}

class CombinedField {
 public:
  MultiObjectCounter<FIELD_WIDTH, FIELD_HEIGHT> counter{fieldSettings()};

  // Each combined observation needs a new frame from both sensors.
  // Discard an old half rather than treating a stopped sensor as an empty view.
  FrameEvents submit(bool leftSensor, const int rawDistances[],
                     const bool rawValid[], std::uint32_t now) {
    const int side = leftSensor ? 0 : 1;
    for (int row = 0; row < FIELD_HEIGHT; ++row) {
      for (int column = 0; column < 8; ++column) {
        const int source = rawZone(row, column, leftSensor);
        const int target = row * FIELD_WIDTH + side * 8 + column;
        distances[target] = rawDistances[source];
        valid[target] = rawValid[source];
      }
    }
    pending[side] = true;
    frameAt[side] = now;
    for (int i = 0; i < 2; ++i) {
      if (pending[i] && now - frameAt[i] > PAIR_MAX_AGE_MS) { pending[i] = false; }
    }
    if (!pending[0] || !pending[1]) { return FrameEvents{}; }
    pending[0] = false;
    pending[1] = false;
    return counter.processFrame(distances, valid, now);
  }

 private:
  int distances[FIELD_CELLS] = {};
  bool valid[FIELD_CELLS] = {};
  bool pending[2] = {};
  std::uint32_t frameAt[2] = {};
};
} // namespace lidar
