#pragma once

#include <cmath>
#include <cstdint>

namespace direction_counter {
// Distance thresholds use millimeters; motion thresholds use grid cells and seconds.
const int GRID_SIZE = 8;
const int ZONE_COUNT = GRID_SIZE * GRID_SIZE;
const int CALIBRATION_FRAMES = 15;
const int OBJECT_DELTA_MM = 150;
const int MAX_OBJECT_DISTANCE_MM = 1200;
const int MIN_OBJECT_CELLS = 3;
const int HISTORY_SIZE = 6; // Maximum history; do not wait for it to fill before estimating.
const float MIN_HORIZONTAL_SPEED = 1.5f;
const float MIN_HORIZONTAL_TRAVEL = 1.0f;
const int DIRECTION_CONFIRM_FRAMES = 2;
const int LOST_RESET_FRAMES = 5;

enum Direction { NONE, LEFT, RIGHT };
enum MovementAxis { COLUMNS, ROWS };

struct DirectionConfig {
  MovementAxis axis = COLUMNS;
  bool reverse = false;
  unsigned long frameTimeoutMs = 500;
};

class DirectionCounter {
 public:
  explicit DirectionCounter(DirectionConfig settings = {}) : config(settings) {}
  bool objectPresent() const { return present; }
  bool isCalibrated() const { return calibrationFrame >= CALIBRATION_FRAMES; }
  unsigned long getLeftCount() const { return leftCount; }
  unsigned long getRightCount() const { return rightCount; }

  // REQUIRES: Both arrays contain ZONE_COUNT entries; now is in milliseconds.
  // MODIFIES: Background, tracking history, and counts.
  // EFFECTS: Returns a new confirmed direction once per object, otherwise NONE.
  Direction processFrame(const int distances[], const bool valid[], unsigned long now) {
    if (hasFrame && now - lastFrameAt > config.frameTimeoutMs) {
      resetTracking(); // A long sensor gap must not join unrelated movements.
    }
    lastFrameAt = now;
    hasFrame = true;
    present = false;
    // Learn per-cell background distances before attempting direction detection.
    if (!isCalibrated()) {
      learnBackground(distances, valid);
      return NONE;
    }

    float centroid = 0.0f;
    if (!findCentroid(distances, valid, centroid)) {
      clearDirectionEvidence(); // Direction estimates must come from uninterrupted observations.
      ++lostFrames;
      if (lostFrames >= LOST_RESET_FRAMES) {
        // Several missing detections rearm the counter for the next object.
        resetTracking();
      }
      return NONE;
    }

    present = true;
    lostFrames = 0;
    addHistory(centroid, now);
    // Two agreeing estimates can confirm a fast passage on its third observation.
    if (historyCount >= 2) {
      return confirmDirection(estimateDirection());
    }
    return NONE;
  }

 private:
  // Average valid startup samples separately for each grid cell.
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
      // Only frequently observed cells receive a reliable background distance.
      baselineValid[zone] = baselineSamples[zone] >= CALIBRATION_FRAMES / 2;
      if (baselineValid[zone]) {
        baseline[zone] = static_cast<float>(baselineSum[zone]) / baselineSamples[zone];
      }
    }
  }

  bool findCentroid(const int distances[], const bool valid[], float &centroid) const {
    int objectCells = 0;
    double weightedX = 0.0;
    double totalWeight = 0.0;
    for (int zone = 0; zone < ZONE_COUNT; ++zone) {
      if (!valid[zone]) {
        continue;
      }
      const float distance = distances[zone];
      bool isObject;
      // Use the learned background when available, otherwise apply a near-range limit.
      if (baselineValid[zone]) {
        isObject = baseline[zone] - distance >= OBJECT_DELTA_MM;
      } else {
        isObject = distance <= MAX_OBJECT_DISTANCE_MM;
      }
      if (isObject) {
        // Nearby cells contribute more to the center along the configured motion axis.
        const double weight = 1000.0 / (distance + 1.0);
        int position = zone % GRID_SIZE;
        if (config.axis == ROWS) { position = zone / GRID_SIZE; }
        if (config.reverse) { position = GRID_SIZE - 1 - position; }
        weightedX += position * weight;
        totalWeight += weight;
        ++objectCells;
      }
    }
    if (objectCells < MIN_OBJECT_CELLS || totalWeight <= 0.0f) {
      return false;
    }
    centroid = static_cast<float>(weightedX / totalWeight);
    return true;
  }

  void addHistory(float x, unsigned long now) {
    if (historyCount == HISTORY_SIZE) {
      // Drop the oldest position and its timestamp together when the window is full.
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

  // REQUIRES: The history contains at least two positions.
  // EFFECTS: Returns the fitted slope along the configured axis in cells per second.
  float horizontalSpeed() const {
    float times[HISTORY_SIZE] = {};
    float meanTime = 0.0f;
    float meanX = 0.0f;
    for (int i = 0; i < historyCount; ++i) {
      // Subtract before converting to float to preserve timer precision.
      times[i] = static_cast<float>(timeHistory[i] - timeHistory[0]) / 1000.0f;
      meanTime += times[i];
      meanX += xHistory[i];
    }
    meanTime /= historyCount;
    meanX /= historyCount;

    float numerator = 0.0f;
    float denominator = 0.0f;
    for (int i = 0; i < historyCount; ++i) {
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
    const float travel = xHistory[historyCount - 1] - xHistory[0];
    const float latestStep = xHistory[historyCount - 1] - xHistory[historyCount - 2];
    const float speed = horizontalSpeed();
    if (std::fabs(travel) < MIN_HORIZONTAL_TRAVEL ||
        std::fabs(speed) < MIN_HORIZONTAL_SPEED) {
      return NONE;
    }
    // Early decisions require the fitted slope, net travel, and newest step to agree.
    if (speed > 0.0f && travel > 0.0f && latestStep > 0.0f) {
      return RIGHT;
    }
    if (speed < 0.0f && travel < 0.0f && latestStep < 0.0f) {
      return LEFT;
    }
    return NONE;
  }

  // Require repeated agreement before latching a single direction event.
  Direction confirmDirection(Direction direction) {
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
      return NONE;
    }
    if (candidate == LEFT) { ++leftCount; }
    if (candidate == RIGHT) { ++rightCount; }
    countedCurrentObject = true;
    return candidate; // Emit once; unchanged following frames return NONE.
  }

  // Clear pending motion without rearming an object that has already been counted.
  void clearDirectionEvidence() {
    historyCount = 0;
    candidate = NONE;
    directionStreak = 0;
  }

  void resetTracking() {
    clearDirectionEvidence();
    lostFrames = 0;
    countedCurrentObject = false;
  }

  DirectionConfig config;
  bool present = false;
  bool hasFrame = false;
  unsigned long lastFrameAt = 0;
  // The per-sensor background stays fixed after startup calibration.
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
