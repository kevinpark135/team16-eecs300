#include "lidarCommon.hpp"
#include "directionSettings.hpp"

SparkFun_VL53L5CX sensor1;
SparkFun_VL53L5CX sensor2;
VL53L5CX_ResultsData data1;
VL53L5CX_ResultsData data2;
direction_counter::DirectionCounter leftCounter(lidar::leftSettings());
direction_counter::DirectionCounter rightCounter(lidar::rightSettings());
bool sensorsReady = false;

void processSensor(const char *name, SparkFun_VL53L5CX &sensor,
                   VL53L5CX_ResultsData &data,
                   direction_counter::DirectionCounter &counter) {
  if (!sensor.isDataReady() || !sensor.getRangingData(&data)) { return; }
  int distances[lidar::ZONE_COUNT];
  bool valid[lidar::ZONE_COUNT];
  for (int zone = 0; zone < lidar::ZONE_COUNT; ++zone) {
    distances[zone] = data.distance_mm[zone];
    valid[zone] = lidar::validReading(data, zone);
  }
  const direction_counter::Direction event = counter.processFrame(distances, valid, millis());
  if (event == direction_counter::NONE) { return; }
  Serial.print(name);
  // RIGHT means increasing configured coordinates, not physical room-right.
  if (event == direction_counter::RIGHT) { Serial.print(": ENTER candidate"); }
  else { Serial.print(": EXIT candidate"); }
  Serial.print(" positive=");
  Serial.print(counter.getRightCount());
  Serial.print(" negative=");
  Serial.println(counter.getLeftCount());
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  sensorsReady = lidar::startSensorPair(sensor1, sensor2, 400000);
  if (!sensorsReady) {
    Serial.println("Could not start both sensors. Check wiring and reset.");
    return;
  }
  Serial.println("Independent L/R direction test. Keep both views empty for calibration.");
  Serial.println("Axis and ENTER/EXIT signs must be checked on the installed unit.");
}

void loop() {
  if (!sensorsReady) { delay(100); return; }
  processSensor("L", sensor1, data1, leftCounter);
  processSensor("R", sensor2, data2, rightCounter);
  delay(1);
}
