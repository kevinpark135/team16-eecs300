#include "lidarCommon.hpp"
#include "directionSettings.hpp"

// Each sensor has its own latest frame and independent direction counter.
SparkFun_VL53L5CX sensor1;
SparkFun_VL53L5CX sensor2;
VL53L5CX_ResultsData data1;
VL53L5CX_ResultsData data2;
direction_counter::DirectionCounter leftCounter(lidar::leftSettings());
direction_counter::DirectionCounter rightCounter(lidar::rightSettings());
bool sensorsReady = false;
const unsigned long STATUS_INTERVAL_MS = 200; // 5 Hz display; sensor ranging stays at 15 Hz.
const unsigned long FRAME_STALE_MS = 500;
unsigned long lastStatusAt = 0;

// Track frame freshness so old distances can be shown as unavailable.
struct SensorStats {
  unsigned long frames = 0;
  unsigned long lastFrameAt = 0;
};
SensorStats leftStats;
SensorStats rightStats;

void processSensor(SparkFun_VL53L5CX &sensor,
                   VL53L5CX_ResultsData &data,
                   direction_counter::DirectionCounter &counter, SensorStats &stats) {
  // Skip missing or failed frames without waiting for the other sensor.
  if (!sensor.isDataReady() || !sensor.getRangingData(&data)) { return; }
  int distances[lidar::ZONE_COUNT];
  bool valid[lidar::ZONE_COUNT];
  // Pass raw millimeter distances and a separate validity mask to the detector.
  for (int zone = 0; zone < lidar::ZONE_COUNT; ++zone) {
    distances[zone] = data.distance_mm[zone];
    valid[zone] = lidar::validReading(data, zone);
  }
  ++stats.frames;
  stats.lastFrameAt = millis();
  counter.processFrame(distances, valid, stats.lastFrameAt);
}

void printGridRow(const VL53L5CX_ResultsData &data, const SensorStats &stats,
                  int row, unsigned long now) {
  const bool fresh = stats.frames > 0 && now - stats.lastFrameAt <= FRAME_STALE_MS;
  for (int column = 0; column < lidar::GRID_SIZE; ++column) {
    const int zone = row * lidar::GRID_SIZE + column;
    // Distinguish an old frame (--) from an invalid measurement in a fresh frame (X).
    if (!fresh) {
      Serial.print("   -- ");
    } else if (!lidar::validReading(data, zone)) {
      Serial.print("    X ");
    } else {
      // Six characters per cell also keep five-digit distances aligned.
      const int distance = data.distance_mm[zone];
      if (distance < 10000) { Serial.print(' '); }
      lidar::printFormattedDistance(distance);
    }
  }
}

void printRunningCount(const char *name, const direction_counter::DirectionCounter &counter) {
  // Mounting settings map the detector's RIGHT to IN and LEFT to OUT.
  Serial.print(name);
  Serial.print(": IN=");
  Serial.print(counter.getRightCount());
  Serial.print(" OUT=");
  Serial.print(counter.getLeftCount());
}

void printDualGrid(unsigned long now) {
  // Wait for both background models before displaying the distance grids.
  if (!leftCounter.isCalibrated() || !rightCounter.isCalibrated()) {
    Serial.print("[calibration] L=");
    Serial.print(leftCounter.isCalibrated() ? "READY" : "LEARNING");
    Serial.print(" R=");
    Serial.println(rightCounter.isCalibrated() ? "READY" : "LEARNING");
    return;
  }
  Serial.println();
  Serial.println("L (0x30), mm                                     | R (0x32), mm");
  // Display each sensor's latest frame; acquisition stays independent.
  for (int row = 0; row < lidar::GRID_SIZE; ++row) {
    printGridRow(data1, leftStats, row, now);
    Serial.print("| ");
    printGridRow(data2, rightStats, row, now);
    Serial.println();
  }
  // Counts are per-sensor direction events, not deduplicated people counts.
  Serial.print("RUNNING COUNT | ");
  printRunningCount("L", leftCounter);
  Serial.print(" | ");
  printRunningCount("R", rightCounter);
  Serial.println();
  Serial.println();
}

// R recreates both counters, clearing counts and restarting background learning.
void resetCalibration() {
  leftCounter = direction_counter::DirectionCounter(lidar::leftSettings());
  rightCounter = direction_counter::DirectionCounter(lidar::rightSettings());
  leftStats = SensorStats{};
  rightStats = SensorStats{};
  Serial.println("[reset] Keep both views empty; leave the divider fixed.");
}

void setup() {
  // Buffer a grid update so UART transmission can overlap sensor polling.
  Serial.setTxBufferSize(2048);
  Serial.begin(115200);
  delay(1500);
  // Print before driver calls so an initialization stall is visible.
  Serial.println("[boot] dual LiDAR bench test, serial 115200");
  // Give the sensors separate I2C addresses before starting their measurements.
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
  Serial.println("Fix the divider, clear both views, and wait for background calibration to finish.");
  Serial.print("Then L/R 8x8 grids print every ");
  Serial.print(STATUS_INTERVAL_MS);
  Serial.println(" ms: X=invalid, --=unavailable/stale.");
  Serial.println("R resets calibration and IN/OUT counts. IN=configured positive direction; OUT=negative.");
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
  processSensor(sensor1, data1, leftCounter, leftStats);
  processSensor(sensor2, data2, rightCounter, rightStats);
  const unsigned long now = millis();
  // Throttle the serial display while polling the sensors on every loop iteration.
  if (now - lastStatusAt >= STATUS_INTERVAL_MS) {
    lastStatusAt = now;
    printDualGrid(now);
  }
  delay(1);
}
