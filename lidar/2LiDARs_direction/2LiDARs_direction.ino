#include "lidarCommon.hpp"
#include "combinedField.hpp"

// Two latest sensor frames feed one 16x8 field with up to two object tracks.
SparkFun_VL53L5CX sensor1;
SparkFun_VL53L5CX sensor2;
VL53L5CX_ResultsData data1;
VL53L5CX_ResultsData data2;
lidar::CombinedField field;
bool sensorsReady = false;
const bool SHOW_FOV = true;                   // Set true to print the 16x8 distance grid above the counts.
const unsigned long STATUS_INTERVAL_MS = 200; // 5 Hz display; sensor ranging stays at 15 Hz.
const unsigned long FRAME_STALE_MS = 500;
unsigned long lastStatusAt = 0;

// Track frame freshness so old distances can be shown as unavailable.
struct SensorStats
{
  unsigned long frames = 0;
  unsigned long lastFrameAt = 0;
};
SensorStats leftStats;
SensorStats rightStats;

void processSensor(SparkFun_VL53L5CX &sensor,
                   VL53L5CX_ResultsData &data,
                   bool leftSensor, SensorStats &stats)
{
  // Skip missing or failed frames without waiting for the other sensor.
  if (!sensor.isDataReady() || !sensor.getRangingData(&data))
  {
    return;
  }
  int distances[lidar::ZONE_COUNT];
  bool valid[lidar::ZONE_COUNT];
  // Pass raw millimeter distances and a separate validity mask to the detector.
  for (int zone = 0; zone < lidar::ZONE_COUNT; ++zone)
  {
    distances[zone] = data.distance_mm[zone];
    valid[zone] = lidar::validReading(data, zone);
  }
  ++stats.frames;
  stats.lastFrameAt = millis();
  field.submit(leftSensor, distances, valid, stats.lastFrameAt);
}

void printGridRow(const VL53L5CX_ResultsData &data, const SensorStats &stats,
                  int row, unsigned long now, bool leftSensor)
{
  const bool fresh = stats.frames > 0 && now - stats.lastFrameAt <= FRAME_STALE_MS;
  // Rotate L 90 degrees counterclockwise and R 90 degrees clockwise for display.
  // Both raw bottom rows then meet at the center of the combined 16x8 view.
  for (int column = 0; column < lidar::GRID_SIZE; ++column)
  {
    const int zone = lidar::rawZone(row, column, leftSensor);
    // Distinguish an old frame (--) from an invalid measurement in a fresh frame (X).
    if (!fresh)
    {
      Serial.print("   -- ");
    }
    else if (!lidar::validReading(data, zone))
    {
      Serial.print("    X ");
    }
    else
    {
      // Six characters per cell also keep five-digit distances aligned.
      const int distance = data.distance_mm[zone];
      if (distance < 10000)
      {
        Serial.print(' ');
      }
      lidar::printFormattedDistance(distance);
    }
  }
}

void printDualGrid(unsigned long now)
{
  // Wait for the combined background model before displaying the field.
  if (!field.counter.isCalibrated())
  {
    Serial.println("[calibration] 16x8 field LEARNING; both sensors must provide fresh frames");
    return;
  }
  if (SHOW_FOV)
  {
    Serial.println();
    // Display each sensor's latest frame; acquisition stays independent.
    for (int row = 0; row < lidar::GRID_SIZE; ++row)
    {
      printGridRow(data1, leftStats, row, now, true);
      printGridRow(data2, rightStats, row, now, false);
      Serial.println();
    }
  }
  const bool fresh = field.counter.hasProcessedFrame() &&
                     now - field.counter.lastFrameTime() <= FRAME_STALE_MS;
  Serial.printf("RUNNING COUNT | IN: %-15luOUT: %-15luCENTROID: ",
                field.counter.getInCount(), field.counter.getOutCount());
  // Old detections do not describe the currently visible field.
  if (fresh)
  {
    Serial.printf("%-15d", field.counter.detectionCount());
  }
  else
  {
    Serial.printf("%-15s", "--");
  }
  Serial.printf("OCCUPANCY: %ld\n", field.counter.getOccupancy());
  if (SHOW_FOV)
  {
    Serial.println();
  }
}

// R clears the combined count, pending frames, and background learning.
void resetCalibration()
{
  field = lidar::CombinedField{};
  leftStats = SensorStats{};
  rightStats = SensorStats{};
  Serial.println("[reset] Keep the entire field empty; leave the sensor mounting fixed.");
}

void setup()
{
  // Buffer a grid update so UART transmission can overlap sensor polling.
  Serial.setTxBufferSize(2048);
  Serial.begin(115200);
  delay(1500);
  // Print before driver calls so an initialization stall is visible.
  Serial.println("[boot] dual LiDAR bench test, serial 115200");
  // Give the sensors separate I2C addresses before starting their measurements.
  sensorsReady = lidar::startSensorPair(sensor1, sensor2, 400000, true, true);
  if (!sensorsReady)
  {
    Serial.println("Could not start both sensors. Check wiring and reset.");
    return;
  }
  Serial.println("Dual LiDAR bench test: no thermal sensor is used.");
  Serial.print("L: LPn GPIO");
  Serial.print(lidar::LPN_PIN_1);
  Serial.print(". R: LPn GPIO");
  Serial.println(lidar::LPN_PIN_2);
  Serial.println("Keep the entire field empty and wait for background calibration to finish.");
  Serial.print(SHOW_FOV ? "The grid and counts print every " : "Counts print every ");
  Serial.print(STATUS_INTERVAL_MS);
  Serial.println(" ms.");
  if (SHOW_FOV)
  {
    Serial.println("Grid: X=invalid, --=unavailable/stale.");
  }
  Serial.println("r resets counts; R resets calibration and counts. IN=down; OUT=up.");
  Serial.println("Tracks up to two separated regions.");
}

void loop()
{
  if (!sensorsReady)
  {
    const unsigned long now = millis();
    if (now - lastStatusAt >= 2000)
    {
      lastStatusAt = now;
      Serial.println("[error] sensor setup failed; check wiring and reset the board");
    }
    delay(100);
    return;
  }
  while (Serial.available() > 0)
  {
    const char command = static_cast<char>(Serial.read());
    if (command == 'r')
    {
      field.counter.resetCounts();
      Serial.println("[reset] IN=0 OUT=0 OCCUPANCY=0");
    }
    else if (command == 'R')
    {
      resetCalibration();
    }
  }
  // Neither sensor waits for the other to produce a fresh frame.
  processSensor(sensor1, data1, true, leftStats);
  processSensor(sensor2, data2, false, rightStats);
  const unsigned long now = millis();
  // Throttle the serial display while polling the sensors on every loop iteration.
  if (now - lastStatusAt >= STATUS_INTERVAL_MS)
  {
    lastStatusAt = now;
    printDualGrid(now);
  }
  delay(1);
}
