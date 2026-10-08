#include "Arduino.h"
#include <Wire.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

#define EMMISIVITY 0.95
#define TA_SHIFT 8

paramsMLX90640 mlx90640;
const byte MLX90640_address = 0x33;
static float tempValues[32 * 24];
static_assert(sizeof(tempValues) == 3072, "Protocol requires 768 32-bit floats");
static bool sensorReady = false;

// Only called outside packet writes; suppress recurring errors for 2 seconds.
void reportError(const char* message, int status) {
  static bool reported = false;
  static unsigned long lastReport = 0;
  unsigned long now = millis();
  if (!reported || now - lastReport >= 2000UL) {
    Serial.print(message);
    Serial.print(": ");
    Serial.println(status);
    lastReport = now;
    reported = true;
  }
}

void setup() {
  Serial.begin(115200);
  // Initial connection test: SDA GPIO21, SCL GPIO22, fixed 400 kHz.
  if (!Wire.begin(21, 22, 400000)) {
    reportError("I2C initialization failed", -1);
    return;
  }
  Wire.setTimeOut(50); // Bound each ESP32 I2C transaction as well.
  Wire.beginTransmission(MLX90640_address);
  int status = Wire.endTransmission();
  if (status != 0) {
    reportError("MLX90640 not detected at 0x33", status);
    return;
  }

  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0) {
    reportError("EEPROM read failed", status);
    return;
  }
  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0) {
    reportError("Parameter extraction failed", status);
    return;
  }
  // The old 0x14 value was masked to 0x04 by the API: retain 8 Hz.
  status = MLX90640_SetRefreshRate(MLX90640_address, 0x04);
  if (status != 0) {
    reportError("Refresh rate setup failed", status);
    return;
  }
  sensorReady = true;
}

void readTempValues() {
  uint8_t subpages = 0;
  const byte maxAttempts = 6;
  for (byte attempt = 0; attempt < maxAttempts; ++attempt) {
    uint16_t frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, frame);
    if (status < 0) {
      reportError("Frame read failed", status);
      continue;
    }
    // GetFrameData returns the subpage, so both 0 and 1 are successes.
    if (status > 1) {
      reportError("Invalid subpage", status);
      continue;
    }
    float Ta = MLX90640_GetTa(frame, &mlx90640);
    MLX90640_CalculateTo(frame, &mlx90640, EMMISIVITY, Ta - TA_SHIFT, tempValues);
    subpages |= static_cast<uint8_t>(1U << status);
    if (subpages == 0x03) {
      // No logging or sensor operations between header and complete payload.
      const uint8_t header[] = {0xAA, 0xBB};
      Serial.write(header, sizeof(header));
      Serial.write(reinterpret_cast<const uint8_t*>(tempValues), sizeof(tempValues));
      return;
    }
  }
  reportError("Incomplete frame discarded", -10);
}

void loop() {
  if (!sensorReady) {
    delay(100);
    return;
  }
  readTempValues();
  delay(30);
}
