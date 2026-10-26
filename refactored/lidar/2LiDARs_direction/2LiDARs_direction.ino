#include "lidarCommon.hpp"
#include "directionSettings.hpp"

SparkFun_VL53L5CX sensor1;
SparkFun_VL53L5CX sensor2;
VL53L5CX_ResultsData data1;
VL53L5CX_ResultsData data2;
direction_counter::DirectionCounter leftCounter(lidar::leftSettings());
direction_counter::DirectionCounter rightCounter(lidar::rightSettings());
bool sensorsReady = false;
const unsigned long STATUS_INTERVAL_MS = 500;
const unsigned long FRAME_STALE_MS = 500;
unsigned long lastStatusAt = 0;

struct SensorStats {
  unsigned long frames = 0;
  unsigned long readFailures = 0;
  unsigned long lastFrameAt = 0;
  int validCells = 0;
  int nearestMm = 0;
  int farthestMm = 0;
};
SensorStats leftStats;
SensorStats rightStats;

void processSensor(const char *name, SparkFun_VL53L5CX &sensor,
                   VL53L5CX_ResultsData &data,
                   direction_counter::DirectionCounter &counter, SensorStats &stats) {
  if (!sensor.isDataReady()) { return; }
  if (!sensor.getRangingData(&data)) {
    ++stats.readFailures;
    return;
  }
  int distances[lidar::ZONE_COUNT];
  bool valid[lidar::ZONE_COUNT];
  stats.validCells = 0;
  stats.nearestMm = 0;
  stats.farthestMm = 0;
  for (int zone = 0; zone < lidar::ZONE_COUNT; ++zone) {
    distances[zone] = data.distance_mm[zone];
    valid[zone] = lidar::validReading(data, zone);
    if (!valid[zone]) { continue; }
    ++stats.validCells;
    if (stats.nearestMm == 0 || distances[zone] < stats.nearestMm) {
      stats.nearestMm = distances[zone];
    }
    if (distances[zone] > stats.farthestMm) { stats.farthestMm = distances[zone]; }
  }
  ++stats.frames;
  stats.lastFrameAt = millis();
  const direction_counter::Direction event = counter.processFrame(distances, valid, stats.lastFrameAt);
  if (event == direction_counter::NONE) { return; }
  Serial.print("[event] ");
  Serial.print(name);
  // Signs refer to configured grid coordinates until mounting is calibrated.
  if (event == direction_counter::RIGHT) { Serial.println(" POSITIVE"); }
  else { Serial.println(" NEGATIVE"); }
}

void printStatus(const char *name, const SensorStats &stats,
                 const direction_counter::DirectionCounter &counter, unsigned long now) {
  const bool fresh = stats.frames > 0 && now - stats.lastFrameAt <= FRAME_STALE_MS;
  Serial.print('[');
  Serial.print(name);
  Serial.print("] frames=");
  Serial.print(stats.frames);
  Serial.print(" state=");
  if (stats.frames == 0) { Serial.print("WAITING"); }
  else if (!fresh) { Serial.print("STALE"); }
  else { Serial.print("LIVE"); }
  Serial.print(" cal=");
  Serial.print(counter.isCalibrated() ? "READY" : "LEARNING");
  Serial.print(" valid=");
  Serial.print(stats.validCells);
  Serial.print("/64 range_mm=");
  if (stats.validCells > 0) {
    Serial.print(stats.nearestMm);
    Serial.print("..");
    Serial.print(stats.farthestMm);
  } else { Serial.print("--"); }
  Serial.print(" object=");
  if (fresh) { Serial.print(counter.objectPresent() ? "1" : "0"); }
  else { Serial.print("--"); }
  Serial.print(" positive=");
  Serial.print(counter.getRightCount());
  Serial.print(" negative=");
  Serial.print(counter.getLeftCount());
  Serial.print(" readFailures=");
  Serial.println(stats.readFailures);
}

void resetCalibration() {
  leftCounter = direction_counter::DirectionCounter(lidar::leftSettings());
  rightCounter = direction_counter::DirectionCounter(lidar::rightSettings());
  leftStats = SensorStats{};
  rightStats = SensorStats{};
  Serial.println("[reset] Keep both views empty; leave the divider fixed.");
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  // Print before driver calls so an initialization stall is visible.
  Serial.println("[boot] dual LiDAR bench test, serial 115200");
  sensorsReady = lidar::startSensorPair(sensor1, sensor2, 400000, true, true);
  if (!sensorsReady) {
    Serial.println("Could not start both sensors. Check wiring and reset.");
    return;
  }
  Serial.println("Dual LiDAR bench test: no thermal sensor is used.");
  Serial.print("L: LPn GPIO");
  Serial.print(lidar::LPN_PIN_1);
  Serial.print(". R: LPn GPIO");
  Serial.println(lidar::LPN_PIN_2);
  Serial.println("Fix the divider, clear both views, and wait for cal=READY on both.");
  Serial.println("R resets calibration and local direction totals. Signs are not calibrated ENTER/EXIT.");
}

void loop() {
  if (!sensorsReady) {
    const unsigned long now = millis();
    if (now - lastStatusAt >= 2000) {
      lastStatusAt = now;
      Serial.println("[error] sensor setup failed; check wiring and reset the board");
    }
    delay(100);
    return;
  }
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 'r' || command == 'R') { resetCalibration(); }
  }
  // Neither sensor waits for the other to produce a fresh frame.
  processSensor("L", sensor1, data1, leftCounter, leftStats);
  processSensor("R", sensor2, data2, rightCounter, rightStats);
  const unsigned long now = millis();
  if (now - lastStatusAt >= STATUS_INTERVAL_MS) {
    lastStatusAt = now;
    printStatus("L", leftStats, leftCounter, now);
    printStatus("R", rightStats, rightCounter, now);
  }
  delay(1);
}
