#include "../passageCounter.hpp"
#include "../../lidar/directionCounter.hpp"
#include "../../thermal/blobDetector.hpp"
#include <cassert>
#include <cstdint>
#include <iostream>
#include <limits>

using namespace passage;

CounterConfig testConfig() {
  CounterConfig config;
  config.windowMs = 1000;
  config.thermalFreshMs = 500;
  config.lidarFreshMs = 200;
  config.clearMs = 20;
  config.initialOccupancy = 5;
  return config;
}

void sendThermal(PassageCounter &counter, int count, std::uint32_t now) {
  counter.thermalFrame(count, true, now - 1);
  counter.thermalFrame(count, true, now);
}

void frames(PassageCounter &counter, std::uint32_t now, bool present = true) {
  counter.lidarFrame(LEFT_SENSOR, present, NONE, now);
  counter.lidarFrame(RIGHT_SENSOR, present, NONE, now);
}

void start(PassageCounter &counter, int count, Motion left, Motion right,
           std::uint32_t now = 10) {
  sendThermal(counter, count, now);
  counter.lidarFrame(LEFT_SENSOR, true, left, now);
  counter.lidarFrame(RIGHT_SENSOR, true, right, now + 1);
}

Decision finish(PassageCounter &counter, int count, std::uint32_t now = 1010) {
  sendThermal(counter, count, now - 1);
  frames(counter, now);
  return counter.update(now);
}

void onePersonBothDirections() {
  for (Motion motion : {ENTER, EXIT}) {
    PassageCounter counter(testConfig());
    start(counter, 1, motion, motion);
    assert(finish(counter, 1) == COUNTED);
    assert(counter.entries() == (motion == ENTER ? 1UL : 0UL));
    assert(counter.exits() == (motion == EXIT ? 1UL : 0UL));
    assert(counter.occupancy() == (motion == ENTER ? 6 : 4));
  }
}

void twoPeopleAllDirections() {
  for (Motion left : {ENTER, EXIT}) {
    for (Motion right : {ENTER, EXIT}) {
      PassageCounter counter(testConfig());
      start(counter, 2, left, right);
      assert(finish(counter, 2) == COUNTED);
      assert(counter.entries() == static_cast<unsigned long>((left == ENTER) + (right == ENTER)));
      assert(counter.entries() + counter.exits() == 2);
    }
  }
}

void onePersonDisagreementAndMissing() {
  for (Motion right : {EXIT, NONE}) {
    PassageCounter counter(testConfig());
    start(counter, 1, ENTER, right);
    assert(finish(counter, 1) == REJECTED);
    assert(counter.entries() == 0 && counter.exits() == 0);
  }
}

void unsupportedAndInvalidThermal() {
  for (int count : {-1, 0, 3, 4}) {
    PassageCounter counter(testConfig());
    start(counter, count, ENTER, ENTER);
    assert(finish(counter, count) == REJECTED);
  }
  PassageCounter invalid(testConfig());
  start(invalid, 1, ENTER, ENTER);
  invalid.thermalFrame(0, false, 1000);
  frames(invalid, 1010);
  assert(invalid.update(1010) == REJECTED);
}

void staleEvidenceAndLidar() {
  PassageCounter oldThermal(testConfig());
  start(oldThermal, 1, ENTER, ENTER);
  frames(oldThermal, 1010);
  assert(oldThermal.update(1010) == REJECTED);

  PassageCounter oldLidar(testConfig());
  start(oldLidar, 1, ENTER, ENTER);
  sendThermal(oldLidar, 1, 1010);
  assert(oldLidar.update(1010) == REJECTED);
}

void duplicateEventsAndFrames() {
  PassageCounter counter(testConfig());
  start(counter, 1, ENTER, ENTER);
  for (std::uint32_t now = 20; now < 1000; now += 10) {
    sendThermal(counter, 1, now);
    counter.lidarFrame(LEFT_SENSOR, true, ENTER, now);
    counter.lidarFrame(RIGHT_SENSOR, true, ENTER, now);
    assert(counter.update(now) == NO_DECISION);
  }
  assert(finish(counter, 1) == COUNTED);
  for (std::uint32_t now = 1020; now < 3000; now += 10) {
    sendThermal(counter, 2, now);
    counter.lidarFrame(LEFT_SENSOR, true, ENTER, now);
    counter.lidarFrame(RIGHT_SENSOR, true, ENTER, now);
    assert(counter.update(now) == NO_DECISION);
  }
  assert(counter.entries() == 1);
}

void delayedDirectionAndThermal() {
  PassageCounter counter(testConfig());
  counter.lidarFrame(LEFT_SENSOR, true, ENTER, 10);
  sendThermal(counter, 1, 100);
  counter.lidarFrame(RIGHT_SENSOR, true, ENTER, 700);
  assert(finish(counter, 1) == COUNTED);
  assert(counter.entries() == 1);
}

void flickerAndUpgrade() {
  PassageCounter flicker(testConfig());
  start(flicker, 1, ENTER, ENTER);
  flicker.thermalFrame(2, true, 100);
  sendThermal(flicker, 1, 200);
  assert(finish(flicker, 1) == COUNTED);
  assert(flicker.entries() == 1);

  PassageCounter upgrade(testConfig());
  start(upgrade, 1, ENTER, EXIT);
  sendThermal(upgrade, 2, 200);
  assert(finish(upgrade, 1) == COUNTED);
  assert(upgrade.entries() == 1 && upgrade.exits() == 1);
}

void rearmOnlyAfterClear() {
  PassageCounter counter(testConfig());
  start(counter, 1, ENTER, ENTER);
  assert(finish(counter, 1) == COUNTED);
  sendThermal(counter, 0, 1020);
  frames(counter, 1020, false);
  counter.update(1020);
  sendThermal(counter, 0, 1040);
  frames(counter, 1040, false);
  counter.update(1040);
  assert(counter.state() == IDLE);
  start(counter, 1, EXIT, EXIT, 1050);
  assert(finish(counter, 1, 2050) == COUNTED);
  assert(counter.entries() == 1 && counter.exits() == 1 && counter.occupancy() == 5);
}

void earlyClearAndNoEvent() {
  PassageCounter counter(testConfig());
  start(counter, 1, ENTER, ENTER);
  sendThermal(counter, 0, 100);
  frames(counter, 100, false);
  assert(counter.update(100) == NO_DECISION);
  sendThermal(counter, 0, 120);
  frames(counter, 120, false);
  assert(counter.update(120) == COUNTED);

  PassageCounter empty(testConfig());
  sendThermal(empty, 2, 10);
  frames(empty, 10);
  assert(empty.update(10000) == NO_DECISION);
  assert(empty.entries() == 0 && empty.exits() == 0);
}

void twoPeoplePartialEvidence() {
  PassageCounter counter(testConfig());
  start(counter, 2, NONE, EXIT);
  assert(finish(counter, 2, 1011) == COUNTED);
  assert(counter.entries() == 0 && counter.exits() == 1); // Never invent the missing direction.
}

void conflictingAndLateEvent() {
  PassageCounter counter(testConfig());
  start(counter, 2, ENTER, EXIT);
  counter.lidarFrame(LEFT_SENSOR, true, EXIT, 100);
  assert(finish(counter, 2) == REJECTED);

  PassageCounter late(testConfig());
  start(late, 1, ENTER, NONE);
  late.lidarFrame(RIGHT_SENSOR, true, ENTER, 1010);
  assert(finish(late, 1) == REJECTED);
}

void wrapAndFreshnessGap() {
  const std::uint32_t startAt = std::numeric_limits<std::uint32_t>::max() - 100;
  PassageCounter counter(testConfig());
  start(counter, 1, ENTER, ENTER, startAt);
  assert(finish(counter, 1, startAt + 1000) == COUNTED);

  PassageCounter gap(testConfig());
  sendThermal(gap, 1, 10);
  gap.thermalFrame(1, true, 1000); // A stale gap needs a second new frame.
  gap.lidarFrame(LEFT_SENSOR, true, ENTER, 1000);
  gap.lidarFrame(RIGHT_SENSOR, true, ENTER, 1000);
  frames(gap, 1100, false);
  gap.thermalFrame(0, true, 1100);
  gap.thermalFrame(0, true, 1101);
  gap.update(1101);
  frames(gap, 1121, false);
  assert(gap.update(1121) == REJECTED);
}

void rawFramesToFinalCount() {
  for (int expectedPeople : {1, 2}) {
    thermal::DetectorConfig thermalConfig;
    thermalConfig.calibrationFrames = 2;
    thermal::BlobDetector detector(thermalConfig);
    thermal::TemperatureFrame background;
    background.fill(20.0f);
    detector.processFrame(background);
    detector.processFrame(background);

    direction_counter::DirectionCounter left;
    direction_counter::DirectionCounter right;
    int distances[64];
    bool valid[64];
    for (int i = 0; i < 64; ++i) { distances[i] = 2000; valid[i] = true; }
    for (int i = 0; i < 15; ++i) {
      left.processFrame(distances, valid, i * 67);
      right.processFrame(distances, valid, i * 67);
    }
    PassageCounter counter(testConfig());
    thermal::TemperatureFrame warm = background;
    for (int person = 0; person < expectedPeople; ++person) {
      for (int row = 2; row < 4; ++row) {
        for (int column = 2; column < 4; ++column) {
          warm[row * 32 + column + person * 16] = 25.0f;
        }
      }
    }
    for (std::uint32_t frame = 0; frame < 8; ++frame) {
      const std::uint32_t now = 2000 + frame * 67;
      const thermal::FrameResult &result = detector.processFrame(warm);
      std::size_t count = 0;
      assert(result.getBlobCount(count) && count == static_cast<std::size_t>(expectedPeople));
      counter.thermalFrame(static_cast<int>(count), true, now);
      for (int i = 0; i < 64; ++i) { distances[i] = 2000; }
      for (int row = 0; row < 3; ++row) { distances[row * 8 + frame] = 800; }
      const direction_counter::Direction l = left.processFrame(distances, valid, now);
      for (int i = 0; i < 64; ++i) { distances[i] = 2000; }
      int rightColumn = static_cast<int>(frame);
      if (expectedPeople == 2) { rightColumn = 7 - rightColumn; }
      for (int row = 0; row < 3; ++row) { distances[row * 8 + rightColumn] = 800; }
      const direction_counter::Direction r = right.processFrame(distances, valid, now);
      Motion rightMotion = NONE;
      if (r == direction_counter::RIGHT) { rightMotion = ENTER; }
      if (r == direction_counter::LEFT) { rightMotion = EXIT; }
      counter.lidarFrame(LEFT_SENSOR, left.objectPresent(), l == direction_counter::RIGHT ? ENTER : NONE, now);
      counter.lidarFrame(RIGHT_SENSOR, right.objectPresent(), rightMotion, now);
      counter.update(now);
    }
    assert(finish(counter, expectedPeople, 3469) == COUNTED);
    assert(counter.entries() == 1);
    assert(counter.exits() == (expectedPeople == 2 ? 1UL : 0UL));
  }
}

int main() {
  onePersonBothDirections();
  twoPeopleAllDirections();
  onePersonDisagreementAndMissing();
  unsupportedAndInvalidThermal();
  staleEvidenceAndLidar();
  duplicateEventsAndFrames();
  delayedDirectionAndThermal();
  flickerAndUpgrade();
  rearmOnlyAfterClear();
  earlyClearAndNoEvent();
  twoPeoplePartialEvidence();
  conflictingAndLateEvent();
  wrapAndFreshnessGap();
  rawFramesToFinalCount();
  std::cout << "PASS 14 passage test groups, including raw frames to final count\n";
}
