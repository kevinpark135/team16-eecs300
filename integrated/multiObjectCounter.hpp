#pragma once

#include "directionCounter.hpp"
#include <cmath>
#include <cstdint>

namespace lidar {
const int MAX_OBJECTS = 2;

struct Detection {
  float x = 0.0f;
  float y = 0.0f;
  float distanceMm = 0.0f;
  int cells = 0;
  int minX = 0;
  int maxX = 0;
  int minY = 0;
  int maxY = 0;
};

struct MultiObjectConfig {
  direction_counter::DirectionConfig direction;
  float maxMatchDistanceCells = 5.0f;
};

// One frame may contain two events, including an IN and an OUT together.
struct FrameEvents {
  bool processed = false;
  int in = 0;
  int out = 0;
};

struct ObjectTrack {
  bool active = false;
  bool observed = false; // False while the last position is only being retained.
  std::uint32_t id = 0;
  Detection detection;
  int missedFrames = 0;
  std::uint32_t lastSeenAt = 0;
  float velocityX = 0.0f; // Grid cells per millisecond, for association only.
  float velocityY = 0.0f;
  float positions[direction_counter::HISTORY_SIZE] = {};
  std::uint32_t times[direction_counter::HISTORY_SIZE] = {};
  int historyCount = 0;
  direction_counter::Direction candidate = direction_counter::NONE;
  int directionStreak = 0;
  direction_counter::Direction countedDirection = direction_counter::NONE;
};

template <int WIDTH, int HEIGHT>
class MultiObjectCounter {
 public:
  static constexpr int CELL_COUNT = WIDTH * HEIGHT;
  explicit MultiObjectCounter(MultiObjectConfig settings = {}) : config(settings) {}

  bool isCalibrated() const { return calibrationFrame >= direction_counter::CALIBRATION_FRAMES; }
  unsigned long getInCount() const { return inCount; }
  unsigned long getOutCount() const { return outCount; }
  int regionCount() const { return regions; } // All regions meeting the area threshold.
  int detectionCount() const { return detectionsUsed; }
  bool overflow() const { return regions > MAX_OBJECTS; }
  const Detection &detection(int index) const { return detections[index]; }
  const ObjectTrack &track(int index) const { return tracks[index]; }
  bool hasProcessedFrame() const { return hasFrame; }
  std::uint32_t lastFrameTime() const { return lastFrameAt; }

  // REQUIRES: Both arrays contain CELL_COUNT entries; now is milliseconds.
  FrameEvents processFrame(const int distances[], const bool valid[], std::uint32_t now) {
    if (hasFrame && now - lastFrameAt > config.direction.frameTimeoutMs) { clearTracks(); }
    hasFrame = true;
    lastFrameAt = now;
    FrameEvents events;
    events.processed = true;
    regions = 0;
    detectionsUsed = 0;
    if (!isCalibrated()) {
      learnBackground(distances, valid);
      return events;
    }
    makeForeground(distances, valid);
    findRegions(distances);
    for (int i = 0; i < MAX_OBJECTS; ++i) {
      tracks[i].observed = false;
      if (tracks[i].active && now - tracks[i].lastSeenAt > config.direction.frameTimeoutMs) {
        tracks[i] = ObjectTrack{};
      }
    }
    // Extra regions cannot be resolved reliably by a two-object tracker.
    // Keep the largest two for diagnostics, but do not invent count events.
    if (overflow()) {
      for (int i = 0; i < MAX_OBJECTS; ++i) { miss(tracks[i]); }
      return events;
    }

    int assignment[MAX_OBJECTS] = {-1, -1};
    associate(now, assignment);
    bool used[MAX_OBJECTS] = {};
    for (int i = 0; i < MAX_OBJECTS; ++i) {
      if (!tracks[i].active) { continue; }
      if (assignment[i] < 0) {
        miss(tracks[i]);
      } else {
        used[assignment[i]] = true;
        observe(tracks[i], detections[assignment[i]], now, events);
      }
    }
    for (int d = 0; d < detectionsUsed; ++d) {
      if (used[d]) { continue; }
      for (int i = 0; i < MAX_OBJECTS; ++i) {
        if (tracks[i].active) { continue; }
        tracks[i] = ObjectTrack{};
        tracks[i].active = true;
        tracks[i].id = nextId++;
        observe(tracks[i], detections[d], now, events);
        break;
      }
    }
    return events;
  }

 private:
  void learnBackground(const int distances[], const bool valid[]) {
    for (int i = 0; i < CELL_COUNT; ++i) {
      if (valid[i] && distances[i] > 0) {
        baselineSum[i] += distances[i];
        ++baselineSamples[i];
      }
    }
    ++calibrationFrame;
    if (!isCalibrated()) { return; }
    for (int i = 0; i < CELL_COUNT; ++i) {
      baselineValid[i] = baselineSamples[i] >= direction_counter::CALIBRATION_FRAMES / 2;
      if (baselineValid[i]) {
        baseline[i] = static_cast<float>(baselineSum[i]) / baselineSamples[i];
      }
    }
  }

  void makeForeground(const int distances[], const bool valid[]) {
    for (int i = 0; i < CELL_COUNT; ++i) {
      foreground[i] = valid[i] && distances[i] > 0 &&
          (baselineValid[i] ? baseline[i] - distances[i] >= direction_counter::OBJECT_DELTA_MM
                            : distances[i] <= direction_counter::MAX_OBJECT_DISTANCE_MM);
    }
  }

  void findRegions(const int distances[]) {
    int queue[CELL_COUNT];
    // Consuming the mask marks cells as visited; each cell is queued only once.
    for (int start = 0; start < CELL_COUNT; ++start) {
      if (!foreground[start]) { continue; }
      Detection region;
      region.minX = region.maxX = start % WIDTH;
      region.minY = region.maxY = start / WIDTH;
      double sumX = 0.0, sumY = 0.0, totalWeight = 0.0, sumDistance = 0.0;
      int head = 0, tail = 0;
      queue[tail++] = start;
      foreground[start] = false;
      while (head < tail) {
        const int cell = queue[head++];
        const int x = cell % WIDTH, y = cell / WIDTH;
        const double weight = 1000.0 / (distances[cell] + 1.0);
        sumX += x * weight;
        sumY += y * weight;
        totalWeight += weight;
        sumDistance += distances[cell];
        ++region.cells;
        if (x < region.minX) { region.minX = x; }
        if (x > region.maxX) { region.maxX = x; }
        if (y < region.minY) { region.minY = y; }
        if (y > region.maxY) { region.maxY = y; }
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const int nx = x + dx, ny = y + dy;
            if (nx < 0 || nx >= WIDTH || ny < 0 || ny >= HEIGHT) { continue; }
            const int neighbor = ny * WIDTH + nx;
            if (foreground[neighbor]) {
              foreground[neighbor] = false;
              queue[tail++] = neighbor;
            }
          }
        }
      }
      if (region.cells < direction_counter::MIN_OBJECT_CELLS) { continue; }
      region.x = static_cast<float>(sumX / totalWeight);
      region.y = static_cast<float>(sumY / totalWeight);
      region.distanceMm = static_cast<float>(sumDistance / region.cells);
      ++regions;
      // Stable area ranking affects display order, never track identity.
      int position = detectionsUsed;
      if (position == MAX_OBJECTS) {
        position = MAX_OBJECTS - 1;
        if (region.cells <= detections[position].cells) { continue; }
      } else {
        ++detectionsUsed;
      }
      while (position > 0 && region.cells > detections[position - 1].cells) {
        detections[position] = detections[position - 1];
        --position;
      }
      detections[position] = region;
    }
  }

  float matchCost(const ObjectTrack &track, const Detection &d, std::uint32_t now) const {
    const float elapsed = static_cast<float>(now - track.lastSeenAt);
    const float px = track.detection.x + track.velocityX * elapsed;
    const float py = track.detection.y + track.velocityY * elapsed;
    const float dx = d.x - px, dy = d.y - py;
    const float cost = dx * dx + dy * dy;
    const float lastDx = d.x - track.detection.x, lastDy = d.y - track.detection.y;
    const float limit = config.maxMatchDistanceCells;
    // A briefly hidden object may stop instead of following its old velocity.
    // Keep prediction for ranking, but also allow a return near the last sighting.
    return cost <= limit * limit || lastDx * lastDx + lastDy * lastDy <= limit * limit
        ? cost : -1.0f;
  }

  void associate(std::uint32_t now, int result[]) const {
    int bestMatches = -1;
    float bestCost = 0.0f;
    // Enumerate both assignments, including leaving a track unmatched.
    for (int first = -1; first < detectionsUsed; ++first) {
      for (int second = -1; second < detectionsUsed; ++second) {
        if (first >= 0 && first == second) { continue; }
        const int choices[MAX_OBJECTS] = {first, second};
        bool allowed = true;
        int matches = 0;
        float totalCost = 0.0f;
        for (int i = 0; i < MAX_OBJECTS; ++i) {
          if (choices[i] < 0) { continue; }
          if (!tracks[i].active) { allowed = false; break; }
          const float cost = matchCost(tracks[i], detections[choices[i]], now);
          if (cost < 0.0f) { allowed = false; break; }
          ++matches;
          totalCost += cost;
        }
        if (allowed && (matches > bestMatches || (matches == bestMatches && totalCost < bestCost))) {
          bestMatches = matches;
          bestCost = totalCost;
          result[0] = first;
          result[1] = second;
        }
      }
    }
  }

  void clearEvidence(ObjectTrack &track) {
    track.historyCount = 0;
    track.candidate = direction_counter::NONE;
    track.directionStreak = 0;
  }

  void miss(ObjectTrack &track) {
    if (!track.active) { return; }
    track.observed = false;
    clearEvidence(track);
    if (++track.missedFrames >= direction_counter::LOST_RESET_FRAMES) { track = ObjectTrack{}; }
  }

  float motionPosition(const Detection &d) const {
    const bool rows = config.direction.axis == direction_counter::ROWS;
    const float position = rows ? d.y : d.x;
    return config.direction.reverse ? (rows ? HEIGHT : WIDTH) - 1 - position : position;
  }

  void observe(ObjectTrack &track, const Detection &d, std::uint32_t now, FrameEvents &events) {
    if (track.historyCount > 0 && now != track.lastSeenAt) {
      const float elapsed = static_cast<float>(now - track.lastSeenAt);
      track.velocityX = (d.x - track.detection.x) / elapsed;
      track.velocityY = (d.y - track.detection.y) / elapsed;
    } else {
      track.velocityX = track.velocityY = 0.0f;
    }
    track.detection = d;
    track.lastSeenAt = now;
    track.missedFrames = 0;
    track.observed = true;
    if (track.historyCount == direction_counter::HISTORY_SIZE) {
      for (int i = 1; i < track.historyCount; ++i) {
        track.positions[i - 1] = track.positions[i];
        track.times[i - 1] = track.times[i];
      }
      --track.historyCount;
    }
    track.positions[track.historyCount] = motionPosition(d);
    track.times[track.historyCount++] = now;
    const auto direction = estimateDirection(track);
    if (direction == direction_counter::NONE) {
      track.candidate = direction_counter::NONE;
      track.directionStreak = 0;
    } else if (direction == track.candidate) {
      if (track.directionStreak < direction_counter::DIRECTION_CONFIRM_FRAMES) { ++track.directionStreak; }
    } else {
      track.candidate = direction;
      track.directionStreak = 1;
    }
    if (track.directionStreak < direction_counter::DIRECTION_CONFIRM_FRAMES ||
        track.countedDirection != direction_counter::NONE) { return; }
    track.countedDirection = direction;
    if (direction == direction_counter::RIGHT) { ++inCount; ++events.in; }
    if (direction == direction_counter::LEFT) { ++outCount; ++events.out; }
  }

  direction_counter::Direction estimateDirection(const ObjectTrack &track) const {
    using namespace direction_counter;
    if (track.historyCount < 2) { return NONE; }
    const int last = track.historyCount - 1;
    const float travel = track.positions[last] - track.positions[0];
    const float step = track.positions[last] - track.positions[last - 1];
    float meanTime = 0.0f, meanPosition = 0.0f;
    for (int i = 0; i <= last; ++i) {
      meanTime += static_cast<float>(track.times[i] - track.times[0]) / 1000.0f;
      meanPosition += track.positions[i];
    }
    meanTime /= track.historyCount;
    meanPosition /= track.historyCount;
    float numerator = 0.0f, denominator = 0.0f;
    for (int i = 0; i <= last; ++i) {
      const float dt = static_cast<float>(track.times[i] - track.times[0]) / 1000.0f - meanTime;
      numerator += dt * (track.positions[i] - meanPosition);
      denominator += dt * dt;
    }
    const float speed = denominator > 0.0f ? numerator / denominator : 0.0f;
    if (std::fabs(travel) < MIN_HORIZONTAL_TRAVEL || std::fabs(speed) < MIN_HORIZONTAL_SPEED) {
      return NONE;
    }
    if (speed > 0.0f && travel > 0.0f && step > 0.0f) { return RIGHT; }
    if (speed < 0.0f && travel < 0.0f && step < 0.0f) { return LEFT; }
    return NONE;
  }

  void clearTracks() {
    for (int i = 0; i < MAX_OBJECTS; ++i) { tracks[i] = ObjectTrack{}; }
  }

  MultiObjectConfig config;
  std::uint32_t baselineSum[CELL_COUNT] = {};
  int baselineSamples[CELL_COUNT] = {};
  float baseline[CELL_COUNT] = {};
  bool baselineValid[CELL_COUNT] = {};
  bool foreground[CELL_COUNT] = {};
  int calibrationFrame = 0;
  Detection detections[MAX_OBJECTS];
  int detectionsUsed = 0;
  int regions = 0;
  ObjectTrack tracks[MAX_OBJECTS];
  std::uint32_t nextId = 1;
  bool hasFrame = false;
  std::uint32_t lastFrameAt = 0;
  unsigned long inCount = 0;
  unsigned long outCount = 0;
};
} // namespace lidar
