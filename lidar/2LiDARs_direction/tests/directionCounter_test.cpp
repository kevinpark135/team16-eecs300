#include "../directionCounter.hpp"
#include "../directionSettings.hpp"
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace direction_counter;

namespace {
void require(bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

struct Scene {
  DirectionCounter counter;
  unsigned long now = 0;

  explicit Scene(DirectionConfig config = {}) : counter(config) {
    for (int i = 0; i < CALIBRATION_FRAMES; ++i) { frame(); }
    require(counter.isCalibrated(), "background did not calibrate");
  }

  Direction frame(float position = -1.0f, unsigned long elapsed = 67,
                  int cells = 3, bool validData = true) {
    std::array<int, ZONE_COUNT> distances;
    std::array<bool, ZONE_COUNT> valid;
    distances.fill(2000);
    valid.fill(validData);
    if (position >= 0.0f) {
      const int column = static_cast<int>(position);
      const float fraction = position - column;
      for (int row = 2; row < 2 + cells; ++row) {
        distances[row * GRID_SIZE + column] = fraction > 0.0f ? 100 : 800;
        if (fraction > 0.0f) {
          // Choose weights that place the centroid between adjacent columns.
          distances[row * GRID_SIZE + column + 1] =
              static_cast<int>(std::lround(101.0f * (1.0f - fraction) / fraction - 1.0f));
        }
      }
    }
    now += elapsed;
    return counter.processFrame(distances.data(), valid.data(), now);
  }
};

void fastPassage(Scene &scene, int first, int second, int third, Direction expected) {
  require(scene.frame(first) == NONE, "one observation emitted a direction");
  require(scene.frame(second) == NONE, "two observations emitted a direction");
  require(scene.frame(third) == expected, "three-observation passage was missed or reversed");
}
} // namespace

int main() {
  int passed = 0;
  const char *current = "";
  auto test = [&](const char *name, auto body) {
    current = name;
    body();
    ++passed;
  };
  try {
    test("fast right passage", [] {
      Scene scene;
      const unsigned long before = scene.now;
      fastPassage(scene, 1, 3, 5, RIGHT);
      require(scene.counter.getRightCount() == 1, "right count was not updated");
      require(scene.now - before == 201, "unexpected synthetic frame timing");
    });
    test("fast left passage", [] {
      Scene scene;
      fastPassage(scene, 6, 4, 2, LEFT);
      require(scene.counter.getLeftCount() == 1, "left count was not updated");
    });
    test("reversed left mounting", [] {
      Scene scene(lidar::leftSettings());
      fastPassage(scene, 6, 4, 2, RIGHT);
    });
    test("right mounting", [] {
      Scene scene(lidar::rightSettings());
      fastPassage(scene, 1, 3, 5, RIGHT);
    });
    test("stationary object", [] {
      Scene scene;
      for (int i = 0; i < 20; ++i) {
        require(scene.frame(3) == NONE, "stationary object was counted");
      }
    });
    test("alternating jitter", [] {
      Scene scene;
      for (int i = 0; i < 20; ++i) {
        require(scene.frame(3 + i % 2) == NONE, "alternating jitter was counted");
      }
    });
    test("reversal before confirmation", [] {
      Scene scene;
      require(scene.frame(1) == NONE, "unexpected first event");
      require(scene.frame(4) == NONE, "unexpected second event");
      require(scene.frame(2) == NONE, "a reversing newest step confirmed stale motion");
    });
    test("two-observation passage stays unconfirmed", [] {
      Scene scene;
      require(scene.frame(0) == NONE, "unexpected first event");
      require(scene.frame(7) == NONE, "two observations should stay unconfirmed");
      for (int i = 0; i < 5; ++i) { scene.frame(); }
      require(scene.counter.getRightCount() == 0, "incomplete passage was counted");
    });
    test("invalid frame breaks direction evidence", [] {
      Scene scene;
      scene.frame(1);
      scene.frame(3);
      scene.frame(-1, 67, 3, false);
      fastPassage(scene, 4, 5, 6, RIGHT);
    });
    test("count once while an object remains visible", [] {
      Scene scene;
      fastPassage(scene, 1, 3, 5, RIGHT);
      for (int column : {6, 7, 6, 5, 4, 3, 2}) {
        require(scene.frame(column) == NONE, "visible object was counted again");
      }
      require(scene.counter.getRightCount() == 1 && scene.counter.getLeftCount() == 0,
              "one object changed multiple totals");
    });
    test("brief occlusion does not rearm", [] {
      Scene scene;
      fastPassage(scene, 1, 3, 5, RIGHT);
      scene.frame();
      scene.frame();
      for (int column : {1, 3, 5}) {
        require(scene.frame(column) == NONE, "brief occlusion caused a duplicate count");
      }
    });
    test("cleared scene rearms for the next object", [] {
      Scene scene;
      fastPassage(scene, 1, 3, 5, RIGHT);
      for (int i = 0; i < 5; ++i) { scene.frame(); }
      fastPassage(scene, 6, 4, 2, LEFT);
      require(scene.counter.getRightCount() == 1 && scene.counter.getLeftCount() == 1,
              "next object's count was lost");
    });
    test("sensor timeout clears pending evidence", [] {
      Scene scene;
      scene.frame(1);
      scene.frame(3);
      require(scene.frame(4, 501) == NONE, "sensor gap joined unrelated observations");
      require(scene.frame(5) == NONE, "post-gap evidence confirmed too early");
      require(scene.frame(6) == RIGHT, "fresh post-gap movement was missed");
    });
    test("minimum object area retained", [] {
      Scene scene;
      for (int column : {1, 3, 5, 7}) {
        require(scene.frame(column, 67, 2) == NONE, "two-cell noise was counted");
      }
    });
    test("slower continuous motion retains the longer history", [] {
      Scene scene;
      for (float position : {1.0f, 1.25f, 1.5f, 1.75f, 2.0f}) {
        require(scene.frame(position) == NONE, "subthreshold travel confirmed too early");
      }
      require(scene.frame(2.25f) == RIGHT, "slower subpixel movement was lost");
    });
    test("minimum speed retained", [] {
      Scene scene;
      for (float position : {1.0f, 1.25f, 1.5f, 1.75f, 2.0f, 2.25f}) {
        require(scene.frame(position, 400) == NONE, "below-threshold motion was counted");
      }
    });
  } catch (const std::exception &error) {
    std::cerr << "FAIL: " << current << ": " << error.what() << '\n';
    return 1;
  }
  std::cout << "PASS: " << passed << " synthetic direction checks; fast passage confirmed "
               "on observation 3 (134 ms after observation 1 at 67 ms/frame).\n";
}
