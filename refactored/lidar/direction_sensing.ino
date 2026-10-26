#include "lidarCommon.hpp"
#include "directionCounter.hpp"

SparkFun_VL53L5CX sensor;
VL53L5CX_ResultsData measurementData;
direction_counter::DirectionCounter counter;
bool sensorReady = false;
unsigned long frameNumber = 0;
const int DISPLAY_DIVIDER = 1;

void printFrame() {
  Serial.print("Frame ");
  Serial.println(frameNumber);
  for (int row = 0; row < lidar::GRID_SIZE; ++row) {
    for (int column = 0; column < lidar::GRID_SIZE; ++column) {
      const int zone = row * lidar::GRID_SIZE + column;
      if (lidar::validReading(measurementData, zone)) {
        Serial.print(measurementData.distance_mm[zone]);
      } else {
        Serial.print('X');
      }
      Serial.print('\t');
    }
    Serial.println();
  }
  Serial.print("\nLEFT: ");
  Serial.print(counter.getLeftCount());
  Serial.print("          RIGHT: ");
  Serial.println(counter.getRightCount());
  Serial.println("\n----------------------------------------");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nVL53L5CX Direction Counter");
  if (Wire.begin(lidar::SDA_PIN, lidar::SCL_PIN)) {
    Wire.setClock(400000);
    sensorReady = sensor.begin() && lidar::configureSensor(sensor);
  }
  if (!sensorReady) {
    Serial.println("Sensor setup failed. Check wiring and reset.");
    return;
  }
  Serial.println("Sensor ready. Keep scene still briefly...");
}

void loop() {
  if (!sensorReady) {
    delay(100);
    return;
  }
  if (!sensor.isDataReady() || !sensor.getRangingData(&measurementData)) {
    return;
  }

  int distances[lidar::ZONE_COUNT];
  bool valid[lidar::ZONE_COUNT];
  for (int zone = 0; zone < lidar::ZONE_COUNT; ++zone) {
    distances[zone] = measurementData.distance_mm[zone];
    valid[zone] = lidar::validReading(measurementData, zone);
  }
  const bool wasCalibrated = counter.isCalibrated();
  // The counter uses plain arrays and does not depend on the sensor library.
  counter.processFrame(distances, valid, millis());
  ++frameNumber;
  if (frameNumber % DISPLAY_DIVIDER == 0) {
    printFrame();
  }
  if (!wasCalibrated && counter.isCalibrated()) {
    Serial.println("\nREADY\n");
  }
}
