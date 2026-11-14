#include "../combinedField.hpp"
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}
void near(float actual, float expected) {
  require(std::fabs(actual - expected) < 0.02f, "unexpected centroid coordinate");
}

struct Scene {
  lidar::CombinedField field;
  std::array<int, lidar::FIELD_CELLS> distances;
  std::array<bool, lidar::FIELD_CELLS> valid;
  std::uint32_t now = 0;

  explicit Scene(bool calibrate = true, lidar::MultiObjectConfig config = lidar::fieldSettings()) {
    field.counter = lidar::MultiObjectCounter<16, 8>(config);
    clear();
    if (calibrate) {
      for (int i = 0; i < direction_counter::CALIBRATION_FRAMES; ++i) { frame(); }
      require(field.counter.isCalibrated(), "background did not calibrate");
    }
  }
  void clear() { distances.fill(2000); valid.fill(true); }
  void duplicate(float row, int leftCells = 3, int rightCells = 3,
                 int leftDistance = 800, int rightDistance = 800) {
    object(row, 4, leftCells); object(row, 9, rightCells);
    const int y = static_cast<int>(row);
    for (int x = 4; x < 4 + leftCells; ++x) { distances[y * 16 + x] = leftDistance; }
    for (int x = 9; x < 9 + rightCells; ++x) { distances[y * 16 + x] = rightDistance; }
  }
  lidar::FrameEvents duplicateFrame(float row) { clear(); duplicate(row); return frame(); }
  void object(float row, int column, int cells = 3) {
    const int y = static_cast<int>(row);
    const float fraction = row - y;
    for (int x = column; x < column + cells; ++x) {
      distances[y * lidar::FIELD_WIDTH + x] = fraction > 0.0f ? 100 : 800;
      if (fraction > 0.0f) {
        distances[(y + 1) * lidar::FIELD_WIDTH + x] =
            static_cast<int>(std::lround(101.0f * (1.0f - fraction) / fraction - 1.0f));
      }
    }
  }
  lidar::FrameEvents submit(bool left, std::uint32_t elapsed = 0) {
    now += elapsed;
    std::array<int, 64> raw;
    std::array<bool, 64> rawValid;
    for (int y = 0; y < 8; ++y) {
      for (int x = 0; x < 8; ++x) {
        const int zone = lidar::rawZone(y, x, left);
        const int cell = y * 16 + (left ? 0 : 8) + x;
        raw[zone] = distances[cell];
        rawValid[zone] = valid[cell];
      }
    }
    return field.submit(left, raw.data(), rawValid.data(), now);
  }
  lidar::FrameEvents frame(std::uint32_t elapsed = 67) {
    submit(true, elapsed);
    return submit(false);
  }
  lidar::FrameEvents move(float row, int column = 3, std::uint32_t elapsed = 67) {
    clear(); object(row, column);
    return frame(elapsed);
  }
  lidar::FrameEvents two(float firstRow, float secondRow, int firstCells = 3, int secondCells = 3) {
    clear(); object(firstRow, 2, firstCells); object(secondRow, 11, secondCells);
    return frame();
  }
  const lidar::ObjectTrack &track(std::uint32_t id) const {
    for (int i = 0; i < lidar::MAX_OBJECTS; ++i) {
      const auto &t = field.counter.track(i);
      if (t.active && t.id == id) { return t; }
    }
    throw std::runtime_error("track disappeared");
  }
};

void noEvents(lidar::FrameEvents events) {
  require(events.in == 0 && events.out == 0, "unexpected count event");
}
} // namespace

int main() {
  int passed = 0;
  int failed = 0;
  const char *current = "";
  auto test = [&](const char *name, auto body) {
    current = name;
    try { body(); ++passed; }
    catch (const std::exception &error) { ++failed; std::cerr << "FAIL: " << name << ": " << error.what() << '\n'; }
  };
  try {
    test("occupancy is signed and resets without losing active passage state", [] {
      Scene scene;
      scene.move(6); scene.move(4); scene.move(2);
      require(scene.field.counter.getOccupancy() == -1, "OUT-first occupancy underflowed");
      const auto id = scene.field.counter.track(0).id;
      scene.field.counter.resetCounts();
      require(scene.field.counter.isCalibrated() && scene.field.counter.getInCount() == 0 &&
              scene.field.counter.getOutCount() == 0 && scene.field.counter.getOccupancy() == 0 &&
              scene.track(id).anchorZone == lidar::ZONE_A, "count-only reset lost state");
      noEvents(scene.move(1)); noEvents(scene.move(3));
      require(scene.move(5).in == 1 && scene.field.counter.getOccupancy() == 1,
              "post-reset full passage failed");
    });
    test("count reset in the middle preserves the observed departure", [] {
      Scene scene;
      scene.move(1); scene.move(3);
      const auto id = scene.field.counter.track(0).id;
      scene.field.counter.resetCounts();
      require(scene.move(5).in == 1 && scene.track(id).observed,
              "count reset discarded a passage in progress");
      noEvents(scene.move(6));
      require(scene.field.counter.getInCount() == 1, "reset caused repeated arrival counts");
    });
    test("a duplicate pair confirms before merging and crosses once", [] {
      Scene scene;
      noEvents(scene.duplicateFrame(1));
      require(scene.field.counter.detectionCount() == 2 &&
              scene.field.counter.detection(0).mergePending, "pair was not held for confirmation");
      const auto id = scene.field.counter.track(0).id;
      noEvents(scene.duplicateFrame(2));
      require(scene.field.counter.regionCount() == 2 && scene.field.counter.mergedCount() == 1 &&
              scene.field.counter.detectionCount() == 1, "pair did not merge");
      scene.duplicateFrame(3);
      require(scene.duplicateFrame(5).in == 1 && scene.field.counter.getInCount() == 1,
              "duplicate pair counted more than once or lost its departure");
      require(scene.track(id).observed, "merge replaced the original identity");
    });
    test("merged position and distance use cell-count weights", [] {
      Scene scene;
      for (int i = 0; i < 2; ++i) {
        scene.clear(); scene.duplicate(1, 3, 4, 800, 900); scene.frame();
      }
      const auto &d = scene.field.counter.detection(0);
      require(scene.field.counter.detectionCount() == 1 && d.cells == 7 && d.sensorMask == 3,
              "wrong merged region metadata");
      near(d.x, (5.0f * 3 + 10.5f * 4) / 7);
      near(d.distanceMm, (800.0f * 3 + 900.0f * 4) / 7);
      require(d.minX == 4 && d.maxX == 12, "wrong merged bounds");
    });
    test("different distances prevent merging", [] {
      Scene scene;
      lidar::FrameEvents events;
      for (int y : {1, 3, 5}) {
        scene.clear(); scene.duplicate(y, 3, 3, 800, 1100); events = scene.frame();
        require(scene.field.counter.detectionCount() == 2 && scene.field.counter.mergedCount() == 0,
                "different-distance objects were combined");
      }
      require(events.in == 2, "distance-separated passages were lost");
    });
    test("different rows prevent merging", [] {
      Scene scene;
      for (int i = 0; i < 3; ++i) {
        scene.clear(); scene.object(1, 4); scene.object(4, 9); scene.frame();
      }
      require(scene.field.counter.detectionCount() == 2, "different-row objects merged");
    });
    test("outside the central band two same-distance objects stay separate", [] {
      Scene scene;
      scene.two(1, 1); scene.two(2, 2);
      require(scene.field.counter.detectionCount() == 2 && scene.field.counter.mergedCount() == 0,
              "outer regions merged");
      require(scene.two(5, 5).in == 2, "two outer passages were lost");
    });
    test("merge confirmation requires consecutive observations", [] {
      Scene scene;
      scene.duplicateFrame(1);
      scene.clear(); scene.frame();
      scene.duplicateFrame(1);
      require(scene.field.counter.detectionCount() == 2, "old pair evidence survived a gap");
      scene.duplicateFrame(2);
      require(scene.field.counter.detectionCount() == 1, "fresh confirmation failed");
    });
    test("opposing central motion is not merged", [] {
      Scene scene;
      auto frame = [&](int leftY, int rightY) {
        scene.clear(); scene.object(leftY, 4); scene.object(rightY, 9); return scene.frame();
      };
      frame(1, 6); frame(3, 4); frame(4, 3);
      require(scene.field.counter.detectionCount() == 2, "opposing motion merged");
      const auto events = frame(5, 2);
      require(events.in == 1 && events.out == 1, "opposing passages lost");
    });
    test("established independent tracks are not merged when ranges become similar", [] {
      Scene scene;
      for (int y : {1, 2}) {
        scene.clear(); scene.duplicate(y, 3, 3, 800, 1100); scene.frame();
      }
      const auto a = scene.field.counter.track(0).id, b = scene.field.counter.track(1).id;
      scene.duplicateFrame(3); const auto events = scene.duplicateFrame(5);
      require(scene.field.counter.detectionCount() == 2 && events.in == 2,
              "known separate objects collapsed");
      require(scene.track(a).observed && scene.track(b).observed, "separate identities lost");
    });
    test("a merged track continues on one sensor and can fully return", [] {
      Scene scene;
      scene.duplicateFrame(1); const auto id = scene.field.counter.track(0).id;
      scene.duplicateFrame(2); scene.duplicateFrame(3);
      require(scene.duplicateFrame(5).in == 1, "merged entry missed");
      noEvents(scene.move(6, 4)); noEvents(scene.move(3, 4));
      require(scene.move(1, 4).out == 1 && scene.track(id).observed,
              "single-sensor handoff lost crossing state");
      require(scene.field.counter.getInCount() == 1 && scene.field.counter.getOutCount() == 1,
              "handoff recounted or lost a full return");
    });
    test("late duplicate appearance preserves an existing right track", [] {
      Scene scene;
      scene.move(1, 9); const auto id = scene.field.counter.track(0).id;
      scene.duplicateFrame(2); scene.duplicateFrame(3);
      require(scene.duplicateFrame(5).in == 1 && scene.track(id).observed,
              "late duplicate changed identity or lost departure");
    });
    test("four raw observations merge into two before the tracking limit", [] {
      Scene scene;
      auto frame = [&](int a, int b) {
        scene.clear(); scene.duplicate(a); scene.duplicate(b); return scene.frame();
      };
      noEvents(frame(0, 7)); noEvents(frame(0, 7));
      require(scene.field.counter.regionCount() == 4 && scene.field.counter.mergedCount() == 2 &&
              scene.field.counter.effectiveRegionCount() == 2 && !scene.field.counter.overflow(),
              "raw observations were truncated before merging");
      noEvents(frame(2, 5)); const auto events = frame(5, 2);
      require(events.in == 1 && events.out == 1, "two merged identities lost their crossings");
    });
    test("one duplicate pair plus an outer object leaves two detections", [] {
      Scene scene;
      for (int i = 0; i < 2; ++i) {
        scene.clear(); scene.duplicate(1); scene.object(6, 13); scene.frame();
      }
      require(scene.field.counter.regionCount() == 3 && scene.field.counter.detectionCount() == 2 &&
              !scene.field.counter.overflow(), "raw three-region limit ran before merging");
    });
    test("unchanged duplicate-pair positions cannot prove an identity crossing", [] {
      Scene scene;
      scene.clear(); scene.duplicate(2); scene.duplicate(5); noEvents(scene.frame());
      // Four raw regions exceed the track limit during initial confirmation.
      scene.clear(); scene.duplicate(5); scene.duplicate(2);
      // This is intentionally ambiguous: unchanged positions cannot prove a crossing.
      noEvents(scene.frame());
      require(scene.field.counter.detectionCount() == 2, "pairs were not confirmed");
    });
    test("a fast duplicate and a separate object can cross during merge confirmation", [] {
      Scene scene;
      scene.clear(); scene.duplicate(2); scene.object(2, 13); noEvents(scene.frame());
      scene.clear(); scene.duplicate(5); scene.object(5, 13);
      const auto events = scene.frame();
      require(events.in == 1 && scene.field.counter.getInCount() == 1,
              "observed departure was lost while three raw regions were pending");
      // The separate object had no track in the overflow frame; its passage is not fabricated.
    });
    test("equally plausible merge partners remain separate", [] {
      Scene scene;
      for (int i = 0; i < 3; ++i) {
        scene.clear(); scene.object(1, 4); scene.object(0, 9); scene.object(2, 9);
        noEvents(scene.frame());
      }
      require(scene.field.counter.mergedCount() == 0 && scene.field.counter.overflow(),
              "ambiguous partners were arbitrarily combined");
    });
    test("a connected seam region is already one detection", [] {
      Scene scene;
      scene.move(1, 6); scene.move(3, 6);
      require(scene.move(5, 6).in == 1 && scene.field.counter.detectionCount() == 1 &&
              scene.field.counter.mergedCount() == 0, "connected region was merged twice");
    });
    test("merging can be disabled for comparison", [] {
      auto config = lidar::fieldSettings(); config.mergeEnabled = false;
      Scene scene(true, config);
      scene.duplicateFrame(1); scene.duplicateFrame(3);
      require(scene.duplicateFrame(5).in == 2 && scene.field.counter.detectionCount() == 2,
              "disabled merger still combined detections");
    });
    test("A to middle to A is not a passage", [] {
      Scene scene;
      for (int y : {1, 3, 4, 3, 1}) { noEvents(scene.move(y)); }
      require(scene.field.counter.getInCount() == 0 && scene.field.counter.getOutCount() == 0,
              "partial entry was counted");
    });
    test("B to middle to B is not a passage", [] {
      Scene scene;
      for (int y : {6, 4, 3, 4, 6}) { noEvents(scene.move(y)); }
    });
    test("full round trips count once per actual transition on the same ID", [] {
      Scene scene;
      scene.move(1); const auto id = scene.field.counter.track(0).id;
      scene.move(3); require(scene.move(5).in == 1, "entry missed");
      noEvents(scene.move(6)); scene.move(4); require(scene.move(2).out == 1, "return exit missed");
      noEvents(scene.move(1)); scene.move(3); require(scene.move(5).in == 1, "second full entry missed");
      require(scene.track(id).observed && scene.field.counter.getInCount() == 2 &&
              scene.field.counter.getOutCount() == 1, "round trip duplicated or replaced the track");
    });
    test("a fast direct observed A to B jump can count", [] {
      Scene scene;
      noEvents(scene.move(2)); require(scene.move(5).in == 1, "direct observed passage rejected");
    });
    test("first appearance in the middle does not imply a departure", [] {
      Scene scene;
      noEvents(scene.move(3)); noEvents(scene.move(4)); noEvents(scene.move(6));
      require(scene.move(1).out == 1 && scene.field.counter.getInCount() == 0,
              "middle appearance fabricated an entry");
    });
    test("zone-edge jitter and stationary arrivals do not recount", [] {
      Scene scene;
      scene.move(1); scene.move(5);
      for (float y : {4.75f, 5.0f, 4.75f, 5.0f, 6.0f, 6.0f}) { noEvents(scene.move(y)); }
      require(scene.field.counter.getInCount() == 1 && scene.field.counter.getOutCount() == 0,
              "arrival boundary jitter counted again");
    });
    test("configurable endpoint confirmation waits for both sides", [] {
      auto config = lidar::fieldSettings(); config.zoneConfirmFrames = 2;
      Scene scene(true, config);
      noEvents(scene.move(1)); noEvents(scene.move(2)); noEvents(scene.move(3));
      noEvents(scene.move(5)); require(scene.move(6).in == 1, "confirmed endpoints missed");
    });
    test("merge confirmation cannot bypass configured departure confirmation", [] {
      auto config = lidar::fieldSettings(); config.zoneConfirmFrames = 2;
      Scene scene(true, config);
      scene.clear(); scene.duplicate(2); scene.object(2, 13); noEvents(scene.frame());
      for (int i = 0; i < 3; ++i) {
        scene.clear(); scene.duplicate(5); scene.object(5, 13); noEvents(scene.frame());
      }
      require(scene.field.counter.getInCount() == 0, "unconfirmed departure was promoted by merging");
    });
    test("lost tracks never infer an unobserved endpoint", [] {
      Scene scene;
      scene.move(1); scene.move(3);
      for (int i = 0; i < 5; ++i) { scene.clear(); noEvents(scene.frame()); }
      noEvents(scene.move(5));
      require(scene.field.counter.getInCount() == 0, "missing data invented an arrival");
    });
    test("rotation is a one-to-one mapping with correct sensor edges", [] {
      for (bool left : {false, true}) {
        std::array<bool, 64> seen{};
        for (int y = 0; y < 8; ++y) {
          for (int x = 0; x < 8; ++x) {
            const int zone = lidar::rawZone(y, x, left);
            require(zone >= 0 && zone < 64 && !seen[zone], "bad sensor rotation");
            seen[zone] = true;
          }
        }
      }
      require(lidar::rawZone(0, 0, true) == 7 && lidar::rawZone(7, 7, true) == 56 &&
              lidar::rawZone(0, 0, false) == 56 && lidar::rawZone(7, 7, false) == 7,
              "wrong rotated edges");
    });
    test("startup needs 15 new frame pairs", [] {
      Scene scene(false);
      for (int i = 0; i < 20; ++i) {
        require(!scene.submit(true, 67).processed, "processed an unpaired frame");
      }
      require(!scene.field.counter.isCalibrated(), "one sensor calibrated the field");
      scene.submit(false, 101); // The pending first half is too old.
      require(!scene.field.counter.hasProcessedFrame(), "accepted an expired pair");
      for (int i = 0; i < 14; ++i) { scene.frame(); }
      require(!scene.field.counter.isCalibrated(), "calibrated too soon");
      scene.frame();
      require(scene.field.counter.isCalibrated(), "paired calibration failed");
    });
    test("same-frame IN and OUT are both returned", [] {
      Scene scene;
      noEvents(scene.two(1, 6));
      noEvents(scene.two(3, 4));
      const auto events = scene.two(5, 2);
      require(events.processed && events.in == 1 && events.out == 1, "lost opposite events");
      require(scene.field.counter.getInCount() == 1 && scene.field.counter.getOutCount() == 1,
              "opposite counts were not accumulated");
    });
    test("two objects in the same direction count twice", [] {
      Scene scene;
      scene.two(1, 1); scene.two(3, 3);
      const auto events = scene.two(5, 5);
      require(events.in == 2 && events.out == 0, "lost a simultaneous IN event");
      noEvents(scene.two(6, 6)); noEvents(scene.two(7, 7));
      require(scene.field.counter.getInCount() == 2, "visible objects counted twice");
    });
    test("two objects in one sensor half are independent", [] {
      Scene scene;
      lidar::FrameEvents events;
      for (int y : {1, 3, 5}) {
        scene.clear(); scene.object(y, 0); scene.object(y, 4);
        events = scene.frame();
      }
      require(events.in == 2 && scene.field.counter.detectionCount() == 2,
              "tracking still depends on sensor side");
    });
    test("single object crosses the seam without changing identity", [] {
      Scene scene;
      scene.move(1, 3);
      const auto id = scene.field.counter.track(0).id;
      scene.move(3, 6);
      require(scene.field.counter.detectionCount() == 1, "seam split the object");
      const auto events = scene.move(5, 10);
      require(events.in == 1 && scene.track(id).observed, "lost seam-crossing track");
      scene.move(6, 11); scene.move(7, 12);
      require(scene.field.counter.getInCount() == 1, "handoff counted twice");
    });
    test("single OUT and empty-field rearming", [] {
      Scene scene;
      scene.move(6); scene.move(4);
      require(scene.move(2).out == 1, "single OUT missed");
      for (int i = 0; i < direction_counter::LOST_RESET_FRAMES; ++i) { scene.clear(); scene.frame(); }
      scene.move(1); scene.move(3);
      require(scene.move(5).in == 1, "next single object was not counted");
      require(scene.field.counter.getOutCount() == 1 && scene.field.counter.getInCount() == 1,
              "wrong rearmed counts");
    });
    test("area ranking does not swap track identities", [] {
      Scene scene;
      scene.two(1, 6, 4, 3);
      const auto downId = scene.field.counter.track(0).id;
      const auto upId = scene.field.counter.track(1).id;
      scene.two(3, 4, 3, 4);
      scene.two(5, 2, 4, 3);
      near(scene.track(downId).detection.y, 5);
      near(scene.track(upId).detection.y, 2);
      require(scene.track(downId).countedDirection == direction_counter::RIGHT &&
              scene.track(upId).countedDirection == direction_counter::LEFT, "area order swapped direction");
    });
    test("prediction retains identities when detection order crosses", [] {
      Scene scene;
      scene.clear(); scene.object(0, 5); scene.object(7, 5); scene.frame();
      const auto downId = scene.field.counter.track(0).id;
      const auto upId = scene.field.counter.track(1).id;
      scene.clear(); scene.object(2, 5); scene.object(5, 5); scene.frame();
      scene.clear(); scene.object(5, 5); scene.object(2, 5);
      const auto events = scene.frame();
      near(scene.track(downId).detection.y, 5);
      near(scene.track(upId).detection.y, 2);
      require(events.in == 1 && events.out == 1, "crossed assignment lost directions");
    });
    test("one missing track does not clear the other", [] {
      Scene scene;
      scene.two(1, 6);
      const auto downId = scene.field.counter.track(0).id;
      const auto upId = scene.field.counter.track(1).id;
      scene.move(3, 2);
      require(scene.track(upId).missedFrames == 1, "missing track was not retained");
      require(scene.move(5, 2).in == 1, "missing neighbor interrupted observed track");
      require(scene.track(downId).observed && !scene.track(upId).observed, "wrong observed flags");
    });
    test("one track expires while the other stays visible", [] {
      Scene scene;
      scene.two(1, 6); scene.two(3, 4); scene.two(5, 2);
      const auto keptId = scene.field.counter.track(0).id;
      for (int i = 0; i < 5; ++i) { scene.move(5, 2); }
      require(scene.track(keptId).observed, "visible object expired");
      scene.two(5, 1); scene.two(5, 3);
      require(scene.two(5, 5).in == 1, "new neighbor did not get a fresh track");
      require(scene.field.counter.getInCount() == 2, "neighbor replacement counted incorrectly");
    });
    test("brief missing observations preserve the arrival side", [] {
      Scene scene;
      scene.move(1); scene.move(3); scene.move(5);
      const auto id = scene.field.counter.track(0).id;
      for (int i = 0; i < 2; ++i) { scene.clear(); scene.frame(); }
      noEvents(scene.move(5)); noEvents(scene.move(6)); noEvents(scene.move(7));
      require(scene.track(id).observed && scene.field.counter.getInCount() == 1,
              "brief loss duplicated the object");
    });
    test("an invalid observation cannot fabricate arrival", [] {
      Scene scene;
      scene.move(1); scene.move(3);
      scene.valid.fill(false); noEvents(scene.frame());
      noEvents(scene.move(4));
      require(scene.move(5).in == 1, "observed arrival did not complete the passage");
    });
    test("long sensor gap resets tracks but preserves totals", [] {
      Scene scene;
      scene.move(1); scene.move(3); scene.move(5);
      const auto oldId = scene.field.counter.track(0).id;
      noEvents(scene.move(1, 3, 501));
      require(scene.field.counter.track(0).id != oldId, "long gap kept the old track");
      noEvents(scene.move(3));
      require(scene.move(5).in == 1 && scene.field.counter.getInCount() == 2,
              "new post-gap movement failed");
    });
    test("per-track timeout applies even when other frames arrive", [] {
      Scene scene;
      scene.two(1, 6);
      scene.clear(); scene.frame(300);
      scene.clear(); scene.frame(300);
      require(!scene.field.counter.track(0).active && !scene.field.counter.track(1).active,
              "stale tracks survived less than five very slow frames");
    });
    test("new frames from both sensors are required for every observation", [] {
      Scene scene;
      scene.move(1); scene.move(3);
      scene.clear(); scene.object(5, 3);
      require(!scene.submit(true, 67).processed, "reused old second half");
      require(scene.field.counter.getInCount() == 0, "unpaired half counted");
      require(!scene.submit(false, 101).processed, "accepted stale first half");
      require(scene.submit(true).in == 1, "new pair failed to count");
      require(!scene.submit(true).processed, "reused a consumed second half");
    });
    test("too-distant detections do not extend an old motion history", [] {
      Scene scene;
      scene.move(1, 0); scene.move(3, 0);
      noEvents(scene.move(5, 13));
      require(scene.field.counter.track(0).missedFrames == 1 &&
              scene.field.counter.track(1).observations == 1, "far detection hijacked old track");
    });
    test("stationary, jitter, sideways and short reversal do not count", [] {
      Scene stationary;
      for (int i = 0; i < 20; ++i) { noEvents(stationary.move(3)); }
      Scene jitter;
      for (int i = 0; i < 20; ++i) { noEvents(jitter.move(3 + i % 2)); }
      Scene sideways;
      for (int x : {1, 4, 7, 10, 13}) { noEvents(sideways.move(3, x)); }
      Scene reversal;
      noEvents(reversal.move(1)); noEvents(reversal.move(4)); noEvents(reversal.move(2));
    });
    test("motion without reaching the opposite zone cannot count", [] {
      Scene scene;
      noEvents(scene.move(0)); noEvents(scene.move(4));
      for (int i = 0; i < 5; ++i) { scene.clear(); noEvents(scene.frame()); }
      require(scene.field.counter.getInCount() == 0, "short passage was counted");
    });
    test("slow subpixel movement counts only at the arrival zone", [] {
      Scene scene;
      for (float row = 1.0f; row < 5.0f; row += 0.25f) { noEvents(scene.move(row)); }
      require(scene.move(5.0f).in == 1, "slow zone-to-zone movement missed");
    });
    test("slow complete passages no longer require a minimum speed", [] {
      Scene scene;
      for (float row : {1.0f, 1.25f, 1.5f, 1.75f, 2.0f, 2.25f}) {
        noEvents(scene.move(row, 3, 400));
      }
      noEvents(scene.move(3, 3, 400)); noEvents(scene.move(4, 3, 400));
      require(scene.move(5, 3, 400).in == 1, "slow full passage was rejected by speed");
    });
    test("minimum area applies to each region separately", [] {
      Scene scene;
      scene.clear(); scene.object(2, 1, 2); scene.object(5, 11, 2); scene.frame();
      require(scene.field.counter.regionCount() == 0, "combined two noise regions");
      scene.clear(); scene.object(2, 6, 3); scene.frame();
      require(scene.field.counter.regionCount() == 1, "three seam cells were not combined");
    });
    test("eight-neighbor connectivity and bounding box", [] {
      Scene scene;
      scene.clear();
      for (int i = 0; i < 3; ++i) { scene.distances[(2 + i) * 16 + 6 + i] = 800; }
      scene.frame();
      require(scene.field.counter.detectionCount() == 1, "diagonal cells did not connect");
      const auto &d = scene.field.counter.detection(0);
      near(d.x, 7); near(d.y, 3);
      require(d.cells == 3 && d.minX == 6 && d.maxX == 8 && d.minY == 2 && d.maxY == 4,
              "wrong component bounds");
    });
    test("row edges do not wrap into adjacent components", [] {
      Scene scene;
      scene.clear();
      for (int y = 0; y < 3; ++y) { scene.distances[y * 16 + 15] = 800; }
      for (int y = 3; y < 6; ++y) { scene.distances[y * 16] = 800; }
      scene.frame();
      require(scene.field.counter.detectionCount() == 2, "BFS wrapped at a row edge");
    });
    test("distance weighting and representative distance", [] {
      Scene scene;
      scene.clear(); scene.object(3, 4);
      scene.distances[3 * 16 + 4] = 400;
      scene.frame();
      const auto &d = scene.field.counter.detection(0);
      const float a = 1000.0f / 401.0f, b = 1000.0f / 801.0f;
      near(d.x, (4 * a + 5 * b + 6 * b) / (a + 2 * b));
      near(d.y, 3); near(d.distanceMm, 2000.0f / 3.0f);
    });
    test("invalid and nonpositive cells never become foreground", [] {
      Scene scene;
      scene.clear(); scene.object(3, 4);
      scene.valid[3 * 16 + 4] = false;
      scene.distances[3 * 16 + 5] = 0;
      scene.distances[3 * 16 + 6] = -1;
      scene.frame();
      require(scene.field.counter.regionCount() == 0, "invalid cells formed a region");
    });
    test("unlearned cells use the near-range fallback", [] {
      Scene scene(false);
      scene.valid.fill(false);
      for (int i = 0; i < direction_counter::CALIBRATION_FRAMES; ++i) { scene.frame(); }
      scene.clear(); scene.object(3, 4); scene.frame();
      require(scene.field.counter.detectionCount() == 1, "near-range fallback failed");
      scene.clear(); scene.distances.fill(1300); scene.frame();
      require(scene.field.counter.regionCount() == 0, "fallback accepted distant cells");
    });
    test("different sensor backgrounds are learned per field cell", [] {
      Scene scene(false);
      for (int y = 0; y < 8; ++y) {
        for (int x = 8; x < 16; ++x) { scene.distances[y * 16 + x] = 900; }
      }
      for (int i = 0; i < direction_counter::CALIBRATION_FRAMES; ++i) { scene.frame(); }
      scene.frame();
      require(scene.field.counter.regionCount() == 0, "shorter background became foreground");
      scene.object(2, 11);
      for (int x = 11; x < 14; ++x) { scene.distances[2 * 16 + x] = 700; }
      scene.frame();
      require(scene.field.counter.regionCount() == 1, "per-cell background delta failed");
    });
    test("more than two regions reports overflow and pauses counting", [] {
      Scene scene;
      for (int y : {1, 3, 5}) {
        scene.clear(); scene.object(y, 0, 3); scene.object(y, 5, 4); scene.object(y, 11, 5);
        noEvents(scene.frame());
      }
      require(scene.field.counter.overflow() && scene.field.counter.regionCount() == 3 &&
              scene.field.counter.detectionCount() == 2, "missing overflow diagnostics");
      require(scene.field.counter.detection(0).cells == 5 &&
              scene.field.counter.detection(1).cells == 4, "wrong area ranking");
    });
    test("a full foreground field cannot overrun the BFS queue", [] {
      Scene scene;
      scene.distances.fill(800); scene.frame();
      require(scene.field.counter.regionCount() == 1 &&
              scene.field.counter.detection(0).cells == 128, "full field traversal failed");
    });
    test("reset clears counts, tracking, calibration and pending halves", [] {
      Scene scene;
      scene.move(1); scene.move(3); scene.move(5); scene.submit(true, 67);
      scene.field = lidar::CombinedField{};
      require(!scene.field.counter.isCalibrated() && scene.field.counter.getInCount() == 0 &&
              !scene.field.counter.track(0).active, "reset kept detector state");
      scene.clear();
      require(!scene.submit(false).processed, "reset retained a pending half");
      for (int i = 0; i < 14; ++i) { scene.frame(); }
      require(!scene.field.counter.isCalibrated(), "reset calibration ended early");
      scene.frame();
      require(scene.field.counter.isCalibrated(), "reset calibration failed");
    });
    test("pairing and motion work across the 32-bit timer wrap", [] {
      Scene scene;
      scene.now = UINT32_MAX - 150;
      noEvents(scene.move(1)); noEvents(scene.move(3));
      require(scene.move(5).in == 1, "timer wrap broke direction tracking");
    });
  } catch (const std::exception &error) {
    std::cerr << "FAIL: " << current << ": " << error.what() << '\n';
    return 1;
  }
  std::cout << "PASS: " << passed << " synthetic multi-object checks.\n";
  return failed ? 1 : 0;
}
