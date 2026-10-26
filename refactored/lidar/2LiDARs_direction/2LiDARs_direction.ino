#include "lidarCommon.hpp"
#include "laneCounter.hpp"

SparkFun_VL53L5CX sensor1;
SparkFun_VL53L5CX sensor2;
VL53L5CX_ResultsData data1;
VL53L5CX_ResultsData data2;
bool sensorsReady = false;
lidar::LaneState leftLane;
lidar::LaneState rightLane;

// REQUIRES: 0 <= firstColumn <= lastColumn < GRID_SIZE.
// EFFECTS: Returns the nearest positive distance, or zero if none exists.
int minimumDistance(const VL53L5CX_ResultsData &data,
                    int firstColumn, int lastColumn) {
  int nearest = 0;
  for (int row = 0; row < lidar::GRID_SIZE; ++row) {
    for (int column = firstColumn; column <= lastColumn; ++column) {
      const int distance = data.distance_mm[row * lidar::GRID_SIZE + column];
      if (distance > 0 && (nearest == 0 || distance < nearest)) {
        nearest = distance;
      }
    }
  }
  return nearest;
}

void processLane(const char *name, lidar::LaneState &lane,
                 int firstColumn, int lastColumn, unsigned long now) {
  const int firstDistance = minimumDistance(data1, firstColumn, lastColumn);
  const int secondDistance = minimumDistance(data2, firstColumn, lastColumn);
  const bool firstActive = firstDistance > 0 && firstDistance < lidar::PERSON_THRESHOLD_MM;
  const bool secondActive = secondDistance > 0 && secondDistance < lidar::PERSON_THRESHOLD_MM;
  const lidar::Crossing crossing = lidar::updateLane(lane, firstActive, secondActive, now);
  if (crossing == lidar::NO_CROSSING) {
    return;
  }
  Serial.print(name);
  if (crossing == lidar::ENTERING) {
    Serial.println(": ===> ENTERING (Sensor 1 -> Sensor 2)");
  } else {
    Serial.println(": <=== LEAVING (Sensor 2 -> Sensor 1)");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  sensorsReady = lidar::startSensorPair(sensor1, sensor2, 400000);
  if (!sensorsReady) {
    Serial.println("Could not start both sensors. Check wiring and reset.");
  }
}

void loop() {
  if (!sensorsReady) {
    delay(100);
    return;
  }
  if (!sensor1.isDataReady() || !sensor2.isDataReady()) {
    return;
  }
  if (!sensor1.getRangingData(&data1) || !sensor2.getRangingData(&data2)) {
    return;
  }

  const unsigned long now = millis();
  // Each half of the 8x8 grid keeps its own crossing state.
  processLane("Lane 1 (Left)", leftLane, 0, 3, now);
  processLane("Lane 2 (Right)", rightLane, 4, 7, now);
}
