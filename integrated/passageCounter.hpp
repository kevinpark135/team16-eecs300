#pragma once

#include <cstdint>

namespace passage {
enum Side { LEFT_SENSOR, RIGHT_SENSOR };
enum Motion { NONE, ENTER, EXIT };
enum State { IDLE, COLLECTING, WAIT_CLEAR };
enum Decision { NO_DECISION, COUNTED, REJECTED };

struct CounterConfig {
  std::uint32_t windowMs = 2000;
  std::uint32_t thermalFreshMs = 1500;
  std::uint32_t lidarFreshMs = 500;
  std::uint32_t clearMs = 300;
  int stableThermalFrames = 2;
  long initialOccupancy = 0;
};

class PassageCounter {
 public:
  explicit PassageCounter(CounterConfig settings = {}) : config(settings) {}
  State state() const { return currentState; }
  unsigned long entries() const { return entryCount; }
  unsigned long exits() const { return exitCount; }
  unsigned long rejected() const { return rejectedCount; }
  long occupancy() const {
    return config.initialOccupancy + static_cast<long>(entryCount) - static_cast<long>(exitCount);
  }

  // MODIFIES: Thermal evidence for the current passage.
  // EFFECTS: Accepts only stable, valid counts; 0 is distinct from invalid data.
  void thermalFrame(int count, bool valid, std::uint32_t now) {
    if (!thermalValid || now - thermalAt > config.thermalFreshMs) {
      candidateFrames = 0;
    }
    thermalAt = now;
    thermalValid = valid && count >= 0;
    if (!thermalValid) {
      candidateFrames = 0;
      stableCount = -1;
      clearStarted = false;
      return;
    }
    if (count != candidateCount) {
      candidateCount = count;
      candidateFrames = 0;
    }
    if (candidateFrames < config.stableThermalFrames) { ++candidateFrames; }
    stableCount = -1;
    if (candidateFrames >= config.stableThermalFrames) { stableCount = count; }
    if (currentState == COLLECTING) { collectThermal(now); }
  }

  // EFFECTS: Records a fresh LiDAR frame and its one-shot direction event.
  void lidarFrame(Side side, bool present, Motion motion, std::uint32_t now) {
    const int index = static_cast<int>(side);
    lidarSeen[index] = true;
    lidarAt[index] = now;
    lidarPresent[index] = present;
    if (motion == NONE || currentState == WAIT_CLEAR) { return; }
    if (currentState == IDLE) {
      currentState = COLLECTING;
      startedAt = now;
      directions[0] = NONE;
      directions[1] = NONE;
      observedCount = 0;
      conflicting = false;
      collectThermal(now);
    }
    if (now - startedAt >= config.windowMs) {
      conflicting = true; // Never join an event outside the matching window.
      return;
    }
    if (directions[index] != NONE && directions[index] != motion) {
      conflicting = true;
    }
    directions[index] = motion;
  }

  // MODIFIES: Totals and passage state.
  // EFFECTS: Applies one decision on clear/timeout, then waits for an empty scene.
  Decision update(std::uint32_t now) {
    const bool empty = sceneEmpty(now);
    if (!empty) { clearStarted = false; }
    if (empty && !clearStarted) {
      clearStarted = true;
      clearAt = now;
    }
    const bool cleared = clearStarted && now - clearAt >= config.clearMs;
    if (currentState == WAIT_CLEAR) {
      if (cleared) { currentState = IDLE; }
      return NO_DECISION;
    }
    if (currentState != COLLECTING) { return NO_DECISION; }
    if (!cleared && now - startedAt < config.windowMs) { return NO_DECISION; }
    const Decision result = finish(now);
    currentState = WAIT_CLEAR;
    clearStarted = false;
    return result;
  }

 private:
  bool thermalFresh(std::uint32_t now) const {
    return thermalValid && now - thermalAt <= config.thermalFreshMs;
  }
  bool lidarsFresh(std::uint32_t now) const {
    return lidarSeen[0] && lidarSeen[1] &&
           now - lidarAt[0] <= config.lidarFreshMs &&
           now - lidarAt[1] <= config.lidarFreshMs;
  }
  bool sceneEmpty(std::uint32_t now) const {
    return thermalFresh(now) && stableCount == 0 && lidarsFresh(now) &&
           !lidarPresent[0] && !lidarPresent[1];
  }
  void collectThermal(std::uint32_t now) {
    if (now - startedAt >= config.windowMs) { return; }
    if (!thermalFresh(now) || stableCount < 1) { return; }
    if (stableCount > 2) {
      conflicting = true;
      return;
    }
    // A later stable two-person observation upgrades this passage, never downgrades it.
    if (stableCount > observedCount) { observedCount = stableCount; }
    evidenceAt = thermalAt;
    hasEvidence = true;
  }
  void add(Motion motion) {
    if (motion == ENTER) { ++entryCount; }
    if (motion == EXIT) { ++exitCount; }
  }
  Decision finish(std::uint32_t now) {
    const bool evidenceFresh = hasEvidence && now - evidenceAt <= config.thermalFreshMs;
    bool accepted = !conflicting && evidenceFresh && thermalFresh(now) && lidarsFresh(now);
    if (observedCount == 1) {
      accepted = accepted && directions[0] != NONE && directions[0] == directions[1];
      if (accepted) { add(directions[0]); }
    } else if (observedCount == 2) {
      accepted = accepted && (directions[0] != NONE || directions[1] != NONE);
      if (accepted) {
        add(directions[0]);
        add(directions[1]);
      }
    } else {
      accepted = false;
    }
    hasEvidence = false;
    if (accepted) { return COUNTED; }
    ++rejectedCount;
    return REJECTED;
  }

  CounterConfig config;
  State currentState = IDLE;
  std::uint32_t startedAt = 0;
  Motion directions[2] = {NONE, NONE};
  int observedCount = 0;
  bool conflicting = false;
  bool hasEvidence = false;
  std::uint32_t evidenceAt = 0;
  int candidateCount = -1;
  int candidateFrames = 0;
  int stableCount = -1;
  bool thermalValid = false;
  std::uint32_t thermalAt = 0;
  bool lidarSeen[2] = {false, false};
  bool lidarPresent[2] = {false, false};
  std::uint32_t lidarAt[2] = {0, 0};
  bool clearStarted = false;
  std::uint32_t clearAt = 0;
  unsigned long entryCount = 0;
  unsigned long exitCount = 0;
  unsigned long rejectedCount = 0;
};
} // namespace passage
