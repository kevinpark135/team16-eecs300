#include "lidarCommon.hpp"

// Arduino keeps these driver objects alive between loop() calls.
SparkFun_VL53L5CX sensor1;
SparkFun_VL53L5CX sensor2;
VL53L5CX_ResultsData data1;
VL53L5CX_ResultsData data2;
bool sensorsReady = false;

void printRow(const VL53L5CX_ResultsData &data, int row) {
  for (int column = 0; column < lidar::GRID_SIZE; ++column) {
    const int index = row * lidar::GRID_SIZE + column;
    lidar::printFormattedDistance(data.distance_mm[index]);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  sensorsReady = lidar::startSensorPair(sensor1, sensor2, 100000);
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
  // Never print a previous frame after a failed read.
  if (!sensor1.getRangingData(&data1) || !sensor2.getRangingData(&data2)) {
    return;
  }

  Serial.println("\n      --- SENSOR 1 (0x30) --- | --- SENSOR 2 (0x32) ---\n");
  for (int row = 0; row < lidar::GRID_SIZE; ++row) {
    printRow(data1, row);
    Serial.print(" |  ");
    printRow(data2, row);
    Serial.println();
  }
  Serial.println();
  delay(50);
}
