#include "../directionCounter.hpp"
#include "../laneCounter.hpp"
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
  passObject(counter, false, 3000); // Still the same object until it disappears.
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

void laneOrderAndIsolation() {
  lidar::LaneState left;
  lidar::LaneState right;
  assert(lidar::updateLane(left, true, false, 100) == lidar::NO_CROSSING);
  assert(lidar::updateLane(right, false, true, 110) == lidar::NO_CROSSING);
  assert(lidar::updateLane(left, false, true, 200) == lidar::ENTERING);
  assert(lidar::updateLane(right, true, false, 210) == lidar::LEAVING);
  assert(left.state == lidar::IDLE && right.state == lidar::IDLE);
  assert(lidar::updateLane(left, true, true, 300) == lidar::NO_CROSSING);
  assert(left.state == lidar::IDLE);
}

void laneTimeoutAndWrap() {
  lidar::LaneState lane;
  lidar::updateLane(lane, true, false, 100);
  lidar::updateLane(lane, false, false, 600);
  assert(lane.state == lidar::SENSOR_1_FIRST); // Original timeout is strictly > 500.
  lidar::updateLane(lane, false, false, 601);
  assert(lane.state == lidar::IDLE);

  const unsigned long start = std::numeric_limits<unsigned long>::max() - 50UL;
  lidar::updateLane(lane, false, true, start);
  assert(lidar::updateLane(lane, true, false, start + 100UL) == lidar::LEAVING);
  // Preserve the original priority: a counterpart detection wins over timeout.
  lidar::updateLane(lane, true, false, 1000);
  assert(lidar::updateLane(lane, false, true, 2000) == lidar::ENTERING);
}

int main() {
  directionsAndRearming();
  stationaryNoiseAndSlowMotion();
  backgroundAndDistanceLimits();
  timerPrecisionAndWrap();
  laneOrderAndIsolation();
  laneTimeoutAndWrap();
  std::cout << "PASS 6 LiDAR test groups (direction, noise, background, timers, lanes)\n";
}
