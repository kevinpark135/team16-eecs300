#pragma once

namespace lidar {
const int PERSON_THRESHOLD_MM = 150;
const unsigned long LANE_TIMEOUT_MS = 500;

enum DirectionState { IDLE, SENSOR_1_FIRST, SENSOR_2_FIRST };
enum Crossing { NO_CROSSING, ENTERING, LEAVING };

struct LaneState {
  DirectionState state = IDLE;
  unsigned long startedAt = 0;
};

// MODIFIES: lane.
// EFFECTS: Reports a crossing when the other sensor activates, or resets on timeout.
inline Crossing updateLane(LaneState &lane, bool firstActive, bool secondActive,
                           unsigned long now) {
  switch (lane.state) {
    case IDLE:
      if (firstActive && !secondActive) {
        lane.state = SENSOR_1_FIRST;
        lane.startedAt = now;
      } else if (secondActive && !firstActive) {
        lane.state = SENSOR_2_FIRST;
        lane.startedAt = now;
      }
      break;
    case SENSOR_1_FIRST:
      if (secondActive) {
        lane.state = IDLE;
        return ENTERING;
      }
      if (now - lane.startedAt > LANE_TIMEOUT_MS) {
        lane.state = IDLE;
      }
      break;
    case SENSOR_2_FIRST:
      if (firstActive) {
        lane.state = IDLE;
        return LEAVING;
      }
      if (now - lane.startedAt > LANE_TIMEOUT_MS) {
        lane.state = IDLE;
      }
      break;
  }
  return NO_CROSSING;
}
} // namespace lidar
