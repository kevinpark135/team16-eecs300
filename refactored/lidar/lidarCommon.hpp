#pragma once

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

namespace lidar {
const int GRID_SIZE = 8;
const int ZONE_COUNT = GRID_SIZE * GRID_SIZE;
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int LPN_PIN_1 = 26;
const int LPN_PIN_2 = 19;
const uint8_t SENSOR_1_ADDRESS = 0x30;
const uint8_t SENSOR_2_ADDRESS = 0x32;
const int RANGING_FREQUENCY_HZ = 15;

// MODIFIES: sensor and the I2C bus.
// EFFECTS: Starts 8x8 ranging; returns false if any setting fails.
inline bool configureSensor(SparkFun_VL53L5CX &sensor) {
  return sensor.setResolution(ZONE_COUNT) &&
         sensor.setRangingFrequency(RANGING_FREQUENCY_HZ) &&
         sensor.startRanging();
}

inline bool wakeSensor(SparkFun_VL53L5CX &sensor, int shutdownPin,
                       uint8_t address) {
  digitalWrite(shutdownPin, HIGH);
  delay(200);
  if (!sensor.begin() || !sensor.setAddress(address)) {
    Serial.println("Sensor initialization/address change failed.");
    return false;
  }
  return true;
}

// MODIFIES: Both sensors, GPIO pins, and Wire.
// EFFECTS: Assigns separate addresses before starting either sensor.
inline bool startSensorPair(SparkFun_VL53L5CX &first,
                            SparkFun_VL53L5CX &second,
                            unsigned long busFrequency) {
  pinMode(LPN_PIN_1, OUTPUT);
  pinMode(LPN_PIN_2, OUTPUT);
  digitalWrite(LPN_PIN_1, LOW);
  digitalWrite(LPN_PIN_2, LOW);
  delay(200);

  // LPn keeps the second sensor off the bus during the first address change.
  if (!Wire.begin(SDA_PIN, SCL_PIN)) {
    return false;
  }
  Wire.setClock(busFrequency);
  if (!wakeSensor(first, LPN_PIN_1, SENSOR_1_ADDRESS) ||
      !wakeSensor(second, LPN_PIN_2, SENSOR_2_ADDRESS)) {
    return false;
  }
  return configureSensor(first) && configureSensor(second);
}

inline bool validReading(const VL53L5CX_ResultsData &data, int zone) {
  const int status = data.target_status[zone];
  return data.nb_target_detected[zone] > 0 &&
         (status == 5 || status == 9) && data.distance_mm[zone] > 0;
}

inline void printFormattedDistance(int distance) {
  if (distance < 10) {
    Serial.print("   ");
  } else if (distance < 100) {
    Serial.print("  ");
  } else if (distance < 1000) {
    Serial.print(" ");
  }
  Serial.print(distance);
  Serial.print(' ');
}
} // namespace lidar
