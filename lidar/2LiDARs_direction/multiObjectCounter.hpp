#pragma once

#include "directionCounter.hpp"
#include <cmath>
#include <cstdint>

namespace lidar {
const int MAX_OBJECTS = 2;
enum PassageZone { UNKNOWN_ZONE, ZONE_A, ZONE_B };

struct Detection {
  float x = 0.0f;
  float y = 0.0f;
  float distanceMm = 0.0f;
  int cells = 0;
  int minX = 0;
  int maxX = 0;
  int minY = 0;
  int maxY = 0;
  unsigned int sensorMask = 0; // 1=left half, 2=right half, 3=both.
  bool mergePending = false;
  PassageZone departureZone = UNKNOWN_ZONE; // Observed while confirming a duplicate pair.
};

struct MultiObjectConfig {
  direction_counter::DirectionConfig direction;
  float maxMatchDistanceCells = 5.0f;
  // Initial heuristic settings, not calibrated physical-space measurements.
  bool mergeEnabled = true;
  float mergeCenterHalfWidthCells = 4.0f;
  float mergeMaxYDifferenceCells = 1.0f;
  float mergeMaxDistanceDifferenceMm = 150.0f;
  int mergeConfirmFrames = 2;
  // Position along the configured motion axis, after direction reversal.
  float zoneAEnd = 2.0f;
  float zoneBStart = 5.0f;
  int zoneConfirmFrames = 1;
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
  int observations = 0;
  int independentFrames = 0;
  PassageZone anchorZone = UNKNOWN_ZONE;
  PassageZone candidateZone = UNKNOWN_ZONE;
  int zoneStreak = 0;
  direction_counter::Direction countedDirection = direction_counter::NONE; // Last crossing only.
};

template <int WIDTH, int HEIGHT>
class MultiObjectCounter {
 public:
  static constexpr int CELL_COUNT = WIDTH * HEIGHT;
  static constexpr int MAX_REGIONS = CELL_COUNT / direction_counter::MIN_OBJECT_CELLS;
  static_assert(WIDTH % 2 == 0, "The combined field needs two equal sensor halves");
  explicit MultiObjectCounter(MultiObjectConfig settings = {}) : config(settings) {}

  bool isCalibrated() const { return calibrationFrame >= direction_counter::CALIBRATION_FRAMES; }
  unsigned long getInCount() const { return inCount; }
  unsigned long getOutCount() const { return outCount; }
  long getOccupancy() const { return static_cast<long>(inCount) - static_cast<long>(outCount); }
  // Preserve background and current passage states so visible objects do not restart.
  void resetCounts() { inCount = 0; outCount = 0; }
  int regionCount() const { return regions; } // All regions meeting the area threshold.
  int detectionCount() const { return detectionsUsed; }
  int mergedCount() const { return mergedPairs; }
  int effectiveRegionCount() const { return effectiveRegions; }
  bool overflow() const { return effectiveRegions > MAX_OBJECTS; }
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
    mergedPairs = 0;
    effectiveRegions = 0;
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
    mergeRegions();
    selectDetections();
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
        tracks[i].anchorZone = detections[d].departureZone;
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
        region.sensorMask |= x < WIDTH / 2 ? 1u : 2u;
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
      rawRegions[regions++] = region;
    }
  }

  struct MergeEvidence {
    bool active = false;
    bool updated = false;
    Detection left;
    Detection right;
    int frames = 0;
    PassageZone departureZone = UNKNOWN_ZONE;
    PassageZone candidateZone = UNKNOWN_ZONE;
    int zoneFrames = 0;
    std::uint32_t departureAt = 0;
    bool departureDelivered = false;
  };

  bool mergeCandidate(const Detection &left, const Detection &right) const {
    const float center = (WIDTH - 1) / 2.0f;
    return left.sensorMask == 1 && right.sensorMask == 2 &&
        left.x >= center - config.mergeCenterHalfWidthCells &&
        right.x <= center + config.mergeCenterHalfWidthCells &&
        std::fabs(left.y - right.y) <= config.mergeMaxYDifferenceCells &&
        std::fabs(left.distanceMm - right.distanceMm) <= config.mergeMaxDistanceDifferenceMm;
  }

  int nearestTrack(const Detection &d) const {
    int best = -1;
    float bestCost = 0.0f;
    for (int i = 0; i < MAX_OBJECTS; ++i) {
      if (!tracks[i].active || !(tracks[i].detection.sensorMask & d.sensorMask)) { continue; }
      const float cost = matchCost(tracks[i], d, lastFrameAt);
      if (cost >= 0.0f && (best < 0 || cost < bestCost)) { best = i; bestCost = cost; }
    }
    return best;
  }

  bool protectedPair(const Detection &left, const Detection &right) const {
    const int a = nearestTrack(left), b = nearestTrack(right);
    if (a < 0 || b < 0 || a == b) { return false; }
    // Tentative duplicates can collapse; established independent objects cannot.
    if (tracks[a].independentFrames >= 2 && tracks[b].independentFrames >= 2) { return true; }
    return tracks[a].velocityY * tracks[b].velocityY < 0.0f &&
        std::fabs(tracks[a].velocityY) > 0.001f && std::fabs(tracks[b].velocityY) > 0.001f;
  }

  static float separationSquared(const Detection &a, const Detection &b) {
    const float dx = a.x - b.x, dy = a.y - b.y;
    return dx * dx + dy * dy;
  }

  int updateMergeEvidence(const Detection &left, const Detection &right, PassageZone &departure) {
    int best = -1;
    float bestCost = 0.0f;
    const float limit = config.maxMatchDistanceCells * config.maxMatchDistanceCells;
    for (int i = 0; i < MAX_OBJECTS; ++i) {
      const auto &e = mergeEvidence[i];
      if (!e.active || e.updated) { continue; }
      const float a = separationSquared(left, e.left), b = separationSquared(right, e.right);
      if (a > limit || b > limit) { continue; }
      if (best < 0 || a + b < bestCost) { best = i; bestCost = a + b; }
    }
    if (best < 0) {
      for (int i = 0; i < MAX_OBJECTS; ++i) {
        if (!mergeEvidence[i].active) { best = i; break; }
      }
      if (best < 0) { return 0; }
      mergeEvidence[best] = MergeEvidence{};
    }
    auto &e = mergeEvidence[best];
    if (e.active) {
      const float leftStep = left.y - e.left.y, rightStep = right.y - e.right.y;
      if (leftStep * rightStep < 0.0f && std::fabs(leftStep) > 0.25f && std::fabs(rightStep) > 0.25f) {
        e = MergeEvidence{};
        return -1; // Opposing observations are not duplicate motion.
      }
    }
    e.active = e.updated = true;
    e.left = left;
    e.right = right;
    const PassageZone leftZone = observedZone(left), rightZone = observedZone(right);
    if (e.departureZone == UNKNOWN_ZONE) {
      if (leftZone != rightZone || leftZone == UNKNOWN_ZONE) {
        e.candidateZone = UNKNOWN_ZONE;
        e.zoneFrames = 0;
      } else {
        if (e.candidateZone != leftZone) { e.candidateZone = leftZone; e.zoneFrames = 0; }
        if (++e.zoneFrames >= config.zoneConfirmFrames) {
          e.departureZone = leftZone;
          e.departureAt = lastFrameAt;
        }
      }
    }
    if (e.frames < config.mergeConfirmFrames) { ++e.frames; }
    if (e.frames >= config.mergeConfirmFrames && !e.departureDelivered) {
      if (lastFrameAt - e.departureAt <= config.direction.frameTimeoutMs) { departure = e.departureZone; }
      e.departureDelivered = true;
    }
    return e.frames;
  }

  void consolidateTracks(const Detection &left, const Detection &right) {
    int a = nearestTrack(left), b = nearestTrack(right);
    if (a < 0 || b < 0 || a == b) { return; }
    // Preserve the older identity and its crossing state, not a new third ID.
    if (tracks[b].id < tracks[a].id) { const int temp = a; a = b; b = temp; }
    if (tracks[a].anchorZone == UNKNOWN_ZONE) { tracks[a].anchorZone = tracks[b].anchorZone; }
    else if (tracks[b].anchorZone != UNKNOWN_ZONE && tracks[a].anchorZone != tracks[b].anchorZone) {
      tracks[a].anchorZone = UNKNOWN_ZONE;
    }
    if (tracks[a].countedDirection == direction_counter::NONE) {
      tracks[a].countedDirection = tracks[b].countedDirection;
    }
    tracks[a].candidateZone = UNKNOWN_ZONE;
    tracks[a].zoneStreak = 0;
    tracks[b] = ObjectTrack{};
  }

  static Detection combine(const Detection &a, const Detection &b) {
    Detection d;
    d.cells = a.cells + b.cells;
    d.x = (a.x * a.cells + b.x * b.cells) / d.cells;
    d.y = (a.y * a.cells + b.y * b.cells) / d.cells;
    d.distanceMm = (a.distanceMm * a.cells + b.distanceMm * b.cells) / d.cells;
    d.minX = a.minX < b.minX ? a.minX : b.minX;
    d.maxX = a.maxX > b.maxX ? a.maxX : b.maxX;
    d.minY = a.minY < b.minY ? a.minY : b.minY;
    d.maxY = a.maxY > b.maxY ? a.maxY : b.maxY;
    d.sensorMask = a.sensorMask | b.sensorMask;
    return d;
  }

  void mergeRegions() {
    for (int i = 0; i < MAX_OBJECTS; ++i) { mergeEvidence[i].updated = false; }
    bool consumed[MAX_REGIONS] = {};
    int partner[MAX_REGIONS];
    // Mutual nearest candidates avoid combining several regions with one partner.
    for (int i = 0; i < regions; ++i) {
      partner[i] = -1;
      float bestCost = 0.0f;
      bool tied = false;
      for (int j = 0; j < regions; ++j) {
        const auto &left = rawRegions[rawRegions[i].sensorMask == 1 ? i : j];
        const auto &right = rawRegions[rawRegions[i].sensorMask == 1 ? j : i];
        if (!config.mergeEnabled || !mergeCandidate(left, right) || protectedPair(left, right)) { continue; }
        const float cost = std::fabs(left.y - right.y) +
            std::fabs(left.distanceMm - right.distanceMm) / 1000.0f;
        if (partner[i] < 0 || cost < bestCost - 0.0001f) {
          partner[i] = j; bestCost = cost; tied = false;
        } else if (std::fabs(cost - bestCost) <= 0.0001f) { tied = true; }
      }
      if (tied) { partner[i] = -1; }
    }
    for (int i = 0; i < regions; ++i) {
      const int j = partner[i];
      if (rawRegions[i].sensorMask != 1 || j < 0 || partner[j] != i || consumed[i] || consumed[j]) { continue; }
      PassageZone departure = UNKNOWN_ZONE;
      const int frames = updateMergeEvidence(rawRegions[i], rawRegions[j], departure);
      if (frames < 0) { continue; }
      if (frames >= config.mergeConfirmFrames) {
        consolidateTracks(rawRegions[i], rawRegions[j]);
        rawRegions[i] = combine(rawRegions[i], rawRegions[j]);
        rawRegions[i].departureZone = departure;
        consumed[j] = true;
        ++mergedPairs;
      } else {
        // Do not create a crossing event from a still-ambiguous duplicate pair.
        rawRegions[i].mergePending = rawRegions[j].mergePending = true;
      }
    }
    for (int i = 0; i < MAX_OBJECTS; ++i) {
      if (!mergeEvidence[i].updated) { mergeEvidence[i] = MergeEvidence{}; }
    }
    for (int i = 0; i < regions; ++i) {
      if (!consumed[i]) { rawRegions[effectiveRegions++] = rawRegions[i]; }
    }
  }

  void selectDetections() {
    for (int i = 0; i < effectiveRegions; ++i) {
      const Detection &region = rawRegions[i];
      int position = detectionsUsed;
      if (position == MAX_OBJECTS) {
        position = MAX_OBJECTS - 1;
        if (region.cells <= detections[position].cells) { continue; }
      } else { ++detectionsUsed; }
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

  void miss(ObjectTrack &track) {
    if (!track.active) { return; }
    track.observed = false;
    track.candidateZone = UNKNOWN_ZONE;
    track.zoneStreak = 0;
    if (++track.missedFrames >= direction_counter::LOST_RESET_FRAMES) { track = ObjectTrack{}; }
  }

  float motionPosition(const Detection &d) const {
    const bool rows = config.direction.axis == direction_counter::ROWS;
    const float position = rows ? d.y : d.x;
    return config.direction.reverse ? (rows ? HEIGHT : WIDTH) - 1 - position : position;
  }

  PassageZone observedZone(const Detection &d) const {
    const float position = motionPosition(d);
    return position <= config.zoneAEnd ? ZONE_A : position >= config.zoneBStart ? ZONE_B : UNKNOWN_ZONE;
  }

  void observe(ObjectTrack &track, const Detection &d, std::uint32_t now, FrameEvents &events) {
    if (track.observations > 0 && track.missedFrames == 0 && now != track.lastSeenAt) {
      const float elapsed = static_cast<float>(now - track.lastSeenAt);
      track.velocityX = (d.x - track.detection.x) / elapsed;
      track.velocityY = (d.y - track.detection.y) / elapsed;
    } else { track.velocityX = track.velocityY = 0.0f; }
    track.detection = d;
    track.lastSeenAt = now;
    track.missedFrames = 0;
    track.observed = true;
    if (track.observations < 2) { ++track.observations; }
    if (!d.mergePending && track.independentFrames < 2) { ++track.independentFrames; }
    const PassageZone zone = observedZone(d);
    if (zone == UNKNOWN_ZONE) {
      track.candidateZone = UNKNOWN_ZONE;
      track.zoneStreak = 0;
      return; // Keep the departure side while passing through the middle.
    }
    if (zone != track.candidateZone) { track.candidateZone = zone; track.zoneStreak = 0; }
    if (track.zoneStreak < config.zoneConfirmFrames) { ++track.zoneStreak; }
    if (track.zoneStreak < config.zoneConfirmFrames || zone == track.anchorZone) { return; }
    if (track.anchorZone == UNKNOWN_ZONE) { track.anchorZone = zone; return; }
    if (d.mergePending) { return; } // Retain departure evidence, but postpone crossing decisions.
    if (track.anchorZone == ZONE_A && zone == ZONE_B) {
      ++inCount; ++events.in; track.countedDirection = direction_counter::RIGHT;
    } else if (track.anchorZone == ZONE_B && zone == ZONE_A) {
      ++outCount; ++events.out; track.countedDirection = direction_counter::LEFT;
    }
    // The arrival side becomes the next departure side, allowing a full return.
    track.anchorZone = zone;
  }

  void clearTracks() {
    for (int i = 0; i < MAX_OBJECTS; ++i) {
      tracks[i] = ObjectTrack{};
      mergeEvidence[i] = MergeEvidence{};
    }
  }

  MultiObjectConfig config;
  std::uint32_t baselineSum[CELL_COUNT] = {};
  int baselineSamples[CELL_COUNT] = {};
  float baseline[CELL_COUNT] = {};
  bool baselineValid[CELL_COUNT] = {};
  bool foreground[CELL_COUNT] = {};
  int calibrationFrame = 0;
  Detection rawRegions[MAX_REGIONS];
  MergeEvidence mergeEvidence[MAX_OBJECTS];
  int mergedPairs = 0;
  int effectiveRegions = 0;
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
