#include "../directionCounter.hpp"
#include <cassert>
#include <iostream>
#include <limits>

using direction_counter::DirectionCounter;
const int ZONES = direction_counter::ZONE_COUNT;

void calibrate(DirectionCounter &counter, bool hasBackground = true) {
  int distances[ZONES];
  bool valid[ZONES];
  for (int i = 0; i < ZONES; ++i) {
    distances[i] = 2000;
    valid[i] = hasBackground;
  }
  for (int i = 0; i < direction_counter::CALIBRATION_FRAMES; ++i) {
    counter.processFrame(distances, valid, static_cast<unsigned long>(i * 67));
  }
  assert(counter.isCalibrated());
  assert(counter.getLeftCount() == 0 && counter.getRightCount() == 0);
}

void objectFrame(DirectionCounter &counter, int column, unsigned long now,
                 int cells = 3, int distance = 800) {
  int distances[ZONES] = {};
  bool valid[ZONES] = {};
  for (int row = 0; row < cells; ++row) {
    const int zone = row * direction_counter::GRID_SIZE + column;
    distances[zone] = distance;
    valid[zone] = true;
  }
  counter.processFrame(distances, valid, now);
}

void passObject(DirectionCounter &counter, bool right, unsigned long start,
                unsigned long step = 67) {
  for (int i = 0; i < 8; ++i) {
    int column = i;
    if (!right) { column = 7 - i; }
    objectFrame(counter, column, start + static_cast<unsigned long>(i) * step);
  }
}

void directionsAndRearming() {
  DirectionCounter counter;
  calibrate(counter);
  passObject(counter, true, 2000);
  assert(counter.getRightCount() == 1 && counter.getLeftCount() == 0);
  passObject(counter, false, 2600); // Still the same object until it disappears.
  assert(counter.getRightCount() == 1 && counter.getLeftCount() == 0);
  int distances[ZONES] = {};
  bool valid[ZONES] = {};
  for (int i = 0; i < direction_counter::LOST_RESET_FRAMES; ++i) {
    counter.processFrame(distances, valid, 4000 + i * 67);
  }
  passObject(counter, false, 5000);
  assert(counter.getLeftCount() == 1 && counter.getRightCount() == 1);
}

void stationaryNoiseAndSlowMotion() {
  DirectionCounter stationary;
  calibrate(stationary);
  for (int i = 0; i < 30; ++i) { objectFrame(stationary, 3, 2000 + i * 67); }
  assert(stationary.getLeftCount() == 0 && stationary.getRightCount() == 0);

  DirectionCounter noise;
  calibrate(noise);
  for (int i = 0; i < 8; ++i) { objectFrame(noise, i, 2000 + i * 67, 2); }
  assert(noise.getRightCount() == 0);

  DirectionCounter slow;
  calibrate(slow);
  passObject(slow, true, 2000, 1000);
  assert(slow.getRightCount() == 0);
}

void backgroundAndDistanceLimits() {
  DirectionCounter emptyBackground;
  calibrate(emptyBackground, false);
  passObject(emptyBackground, true, 2000);
  assert(emptyBackground.getRightCount() == 1);

  DirectionCounter distant;
  calibrate(distant, false);
  for (int i = 0; i < 8; ++i) { objectFrame(distant, i, 2000 + i * 67, 3, 1201); }
  assert(distant.getRightCount() == 0);

  DirectionCounter unchanged;
  calibrate(unchanged);
  for (int i = 0; i < 8; ++i) { objectFrame(unchanged, i, 2000 + i * 67, 3, 1851); }
  assert(unchanged.getRightCount() == 0);
}

void timerPrecisionAndWrap() {
  DirectionCounter sameTime;
  calibrate(sameTime);
  passObject(sameTime, true, 2000, 0);
  assert(sameTime.getRightCount() == 0);

  DirectionCounter longUptime;
  calibrate(longUptime);
  passObject(longUptime, true, 4000000000UL);
  assert(longUptime.getRightCount() == 1);

  DirectionCounter wrap;
  calibrate(wrap);
  passObject(wrap, true, std::numeric_limits<unsigned long>::max() - 200UL);
  assert(wrap.getRightCount() == 1);
}

void axesInversionAndEvents() {
  for (direction_counter::MovementAxis axis : {direction_counter::COLUMNS, direction_counter::ROWS}) {
    for (bool reverse : {false, true}) {
      direction_counter::DirectionConfig config;
      config.axis = axis;
      config.reverse = reverse;
      DirectionCounter counter(config);
      calibrate(counter);
      int events = 0;
      for (int position = 0; position < 8; ++position) {
        int distances[ZONES] = {};
        bool valid[ZONES] = {};
        for (int offset = 0; offset < 3; ++offset) {
          int zone = offset * 8 + position;
          if (axis == direction_counter::ROWS) { zone = position * 8 + offset; }
          distances[zone] = 800;
          valid[zone] = true;
        }
        const direction_counter::Direction event = counter.processFrame(distances, valid, 2000 + position * 67);
        if (event != direction_counter::NONE) {
          ++events;
          assert(event == (reverse ? direction_counter::LEFT : direction_counter::RIGHT));
        }
      }
      assert(events == 1 && counter.objectPresent());
      assert(counter.getLeftCount() == (reverse ? 1UL : 0UL));
      assert(counter.getRightCount() == (reverse ? 0UL : 1UL));
    }
  }
}

void oneShotAndSensorGap() {
  DirectionCounter counter;
  calibrate(counter);
  passObject(counter, true, 2000);
  int distances[ZONES] = {};
  bool valid[ZONES] = {};
  for (int row = 0; row < 3; ++row) {
    distances[row * 8 + 7] = 800;
    valid[row * 8 + 7] = true;
  }
  for (int i = 0; i < 10; ++i) {
    assert(counter.processFrame(distances, valid, 2500 + i * 67) == direction_counter::NONE);
  }
  passObject(counter, false, 5000); // A long missing-frame interval starts a fresh history.
  assert(counter.getRightCount() == 1 && counter.getLeftCount() == 1);
}

void independentCounters() {
  DirectionCounter left;
  DirectionCounter right;
  calibrate(left);
  calibrate(right);
  for (int i = 0; i < 8; ++i) {
    objectFrame(left, i, 2000 + i * 67);
    objectFrame(right, 7 - i, 2000 + i * 67);
  }
  assert(left.getRightCount() == 1 && left.getLeftCount() == 0);
  assert(right.getLeftCount() == 1 && right.getRightCount() == 0);
}

int main() {
  directionsAndRearming();
  stationaryNoiseAndSlowMotion();
  backgroundAndDistanceLimits();
  timerPrecisionAndWrap();
  axesInversionAndEvents();
  oneShotAndSensorGap();
  independentCounters();
  std::cout << "PASS 7 LiDAR test groups (direction, background, timers, axes, events, independence)\n";
}
