#pragma once

#include <cmath>
#include <cstdint>

namespace direction_counter {
const int GRID_SIZE = 8;
const int ZONE_COUNT = GRID_SIZE * GRID_SIZE;
const int CALIBRATION_FRAMES = 15;
const int OBJECT_DELTA_MM = 150;
const int MAX_OBJECT_DISTANCE_MM = 1200;
const int MIN_OBJECT_CELLS = 3;
const int HISTORY_SIZE = 6;
const float MIN_HORIZONTAL_SPEED = 1.5f;
const float MIN_HORIZONTAL_TRAVEL = 1.0f;
const int DIRECTION_CONFIRM_FRAMES = 3;
const int LOST_RESET_FRAMES = 5;

enum Direction { NONE, LEFT, RIGHT };

class DirectionCounter {
 public:
  bool isCalibrated() const { return calibrationFrame >= CALIBRATION_FRAMES; }
  unsigned long getLeftCount() const { return leftCount; }
  unsigned long getRightCount() const { return rightCount; }

  // REQUIRES: Both arrays contain ZONE_COUNT entries; now is in milliseconds.
  // MODIFIES: Background, tracking history, and counts.
  // EFFECTS: Learns a fixed background, then counts each tracked object once.
  void processFrame(const int distances[], const bool valid[], unsigned long now) {
    if (!isCalibrated()) {
      learnBackground(distances, valid);
      return;
    }

    float centroid = 0.0f;
    if (!findCentroid(distances, valid, centroid)) {
      ++lostFrames;
      if (lostFrames >= LOST_RESET_FRAMES) {
        resetTracking();
      }
      return;
    }

    lostFrames = 0;
    addHistory(centroid, now);
    if (historyCount == HISTORY_SIZE) {
      confirmDirection(estimateDirection());
    }
  }

 private:
  void learnBackground(const int distances[], const bool valid[]) {
    for (int zone = 0; zone < ZONE_COUNT; ++zone) {
      if (valid[zone]) {
        baselineSum[zone] += distances[zone];
        ++baselineSamples[zone];
      }
    }
    ++calibrationFrame;
    if (!isCalibrated()) {
      return;
    }
    for (int zone = 0; zone < ZONE_COUNT; ++zone) {
      baselineValid[zone] = baselineSamples[zone] >= CALIBRATION_FRAMES / 2;
      if (baselineValid[zone]) {
        baseline[zone] = static_cast<float>(baselineSum[zone]) / baselineSamples[zone];
      }
    }
  }

  bool findCentroid(const int distances[], const bool valid[], float &centroid) const {
    int objectCells = 0;
    float weightedX = 0.0f;
    float totalWeight = 0.0f;
    for (int zone = 0; zone < ZONE_COUNT; ++zone) {
      if (!valid[zone]) {
        continue;
      }
      const float distance = distances[zone];
      bool isObject;
      if (baselineValid[zone]) {
        isObject = baseline[zone] - distance >= OBJECT_DELTA_MM;
      } else {
        isObject = distance <= MAX_OBJECT_DISTANCE_MM;
      }
      if (isObject) {
        // Nearby cells contribute more to the horizontal center.
        const float weight = 1000.0f / (distance + 1.0f);
        weightedX += (zone % GRID_SIZE) * weight;
        totalWeight += weight;
        ++objectCells;
      }
    }
    if (objectCells < MIN_OBJECT_CELLS || totalWeight <= 0.0f) {
      return false;
    }
    centroid = weightedX / totalWeight;
    return true;
  }

  void addHistory(float x, unsigned long now) {
    if (historyCount == HISTORY_SIZE) {
      for (int i = 0; i < HISTORY_SIZE - 1; ++i) {
        xHistory[i] = xHistory[i + 1];
        timeHistory[i] = timeHistory[i + 1];
      }
      --historyCount;
    }
    xHistory[historyCount] = x;
    timeHistory[historyCount] = now;
    ++historyCount;
  }

  // REQUIRES: The history contains HISTORY_SIZE positions.
  // EFFECTS: Returns the fitted slope in columns per second.
  float horizontalSpeed() const {
    float times[HISTORY_SIZE] = {};
    float meanTime = 0.0f;
    float meanX = 0.0f;
    for (int i = 0; i < HISTORY_SIZE; ++i) {
      // Subtract before converting to float to preserve timer precision.
      times[i] = static_cast<float>(timeHistory[i] - timeHistory[0]) / 1000.0f;
      meanTime += times[i];
      meanX += xHistory[i];
    }
    meanTime /= HISTORY_SIZE;
    meanX /= HISTORY_SIZE;

    float numerator = 0.0f;
    float denominator = 0.0f;
    for (int i = 0; i < HISTORY_SIZE; ++i) {
      const float dt = times[i] - meanTime;
      numerator += dt * (xHistory[i] - meanX);
      denominator += dt * dt;
    }
    if (denominator == 0.0f) {
      return 0.0f;
    }
    return numerator / denominator;
  }

  Direction estimateDirection() const {
    float minX = xHistory[0];
    float maxX = xHistory[0];
    for (int i = 1; i < HISTORY_SIZE; ++i) {
      if (xHistory[i] < minX) { minX = xHistory[i]; }
      if (xHistory[i] > maxX) { maxX = xHistory[i]; }
    }
    const float speed = horizontalSpeed();
    if (maxX - minX < MIN_HORIZONTAL_TRAVEL ||
        std::fabs(speed) < MIN_HORIZONTAL_SPEED) {
      return NONE;
    }
    // Increasing columns are treated as rightward movement.
    if (speed > 0.0f) {
      return RIGHT;
    }
    return LEFT;
  }

  void confirmDirection(Direction direction) {
    if (direction == NONE) {
      candidate = NONE;
      directionStreak = 0;
    } else if (direction == candidate) {
      ++directionStreak;
    } else {
      candidate = direction;
      directionStreak = 1;
    }
    if (directionStreak < DIRECTION_CONFIRM_FRAMES || countedCurrentObject) {
      return;
    }
    if (candidate == LEFT) { ++leftCount; }
    if (candidate == RIGHT) { ++rightCount; }
    countedCurrentObject = true;
  }

  void resetTracking() {
    historyCount = 0;
    candidate = NONE;
    directionStreak = 0;
    lostFrames = 0;
    countedCurrentObject = false;
  }

  std::uint32_t baselineSum[ZONE_COUNT] = {};
  int baselineSamples[ZONE_COUNT] = {};
  float baseline[ZONE_COUNT] = {};
  bool baselineValid[ZONE_COUNT] = {};
  int calibrationFrame = 0;
  float xHistory[HISTORY_SIZE] = {};
  unsigned long timeHistory[HISTORY_SIZE] = {};
  int historyCount = 0;
  Direction candidate = NONE;
  int directionStreak = 0;
  int lostFrames = 0;
  bool countedCurrentObject = false;
  unsigned long leftCount = 0;
  unsigned long rightCount = 0;
};
} // namespace direction_counter
