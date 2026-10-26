#include "Arduino.h"
#include <Wire.h>
#include <math.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "Thermal_Config.h"

const float EMISSIVITY = 0.95f;
const float AMBIENT_SHIFT_C = 8.0f;
const int PIXEL_COUNT = 32 * 24;
const int EEPROM_WORD_COUNT = 832;
const int FRAME_WORD_COUNT = 834;
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const unsigned long I2C_FREQUENCY_HZ = 400000;
const byte REFRESH_RATE_SETTING = 0x04; // Sensor register code for 8 Hz.

paramsMLX90640 mlx90640;
const byte MLX90640_address = 0x33;
static float tempValues[PIXEL_COUNT] = {};
static_assert(sizeof(tempValues) == 3072, "Protocol requires 768 32-bit floats");
static bool sensorReady = false;
static const char *sensorFailureMessage = nullptr;
static int sensorFailureStatus = 0;
static uint32_t completedFrames = 0;

void reportError(const char *message, int status)
{
#if THERMAL_DIAGNOSTIC_MODE
  static unsigned long lastReport = 0;
  static bool reported = false;
  const unsigned long now = millis();
  if (!reported || now - lastReport >= 2000UL)
  {
    Serial.print("[error] ");
    Serial.print(message);
    Serial.print(": ");
    Serial.println(status);
    lastReport = now;
    reported = true;
  }
#else
  (void)message;
  (void)status;
#endif
}

void failSensorInitialization(const char *message, int status)
{
  sensorFailureMessage = message;
  sensorFailureStatus = status;
  reportError(message, status);
}

void reportTemperatureFrame()
{
#if THERMAL_DIAGNOSTIC_MODE
  float minTemperature = INFINITY;
  float maxTemperature = -INFINITY;
  uint16_t finiteCount = 0;
  for (uint16_t i = 0; i < PIXEL_COUNT; ++i)
  {
    if (!isfinite(tempValues[i]))
      continue;
    ++finiteCount;
    minTemperature = min(minTemperature, tempValues[i]);
    maxTemperature = max(maxTemperature, tempValues[i]);
  }

  Serial.print("[temperature] frame=");
  Serial.print(completedFrames);
  Serial.print(" finite=");
  Serial.print(finiteCount);
  Serial.print("/768");
  if (finiteCount > 0)
  {
    Serial.print(" min=");
    Serial.print(minTemperature, 2);
    Serial.print("C max=");
    Serial.print(maxTemperature, 2);
    Serial.print("C");
  }
  Serial.println();
#endif
}

int connectSensor()
{
  const byte maxAttempts = 5;
  int status = -1;

  for (byte attempt = 1; attempt <= maxAttempts; ++attempt)
  {
    if (attempt > 1)
    {
      Wire.end();
      delay(10);
    }

    if (!Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQUENCY_HZ))
    {
      status = -1;
    }
    else
    {
      Wire.setTimeOut(50);
      // The datasheet specifies an 80 ms POR wait before the first access.
      delay(100);
      Wire.beginTransmission(MLX90640_address);
      status = Wire.endTransmission();
    }

#if THERMAL_DIAGNOSTIC_MODE
    Serial.print("[setup] sensor probe attempt=");
    Serial.print(attempt);
    Serial.print(" status=");
    Serial.println(status);
#endif

    if (status == 0)
      return 0;
    delay(100);
  }

  return status;
}

// MODIFIES: Sensor settings and calibration parameters.
// EFFECTS: Returns false if calibration or refresh-rate validation fails.
bool loadCalibrationAndSettings()
{
  uint16_t eeMLX90640[EEPROM_WORD_COUNT];
  int status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0)
  {
    failSensorInitialization("EEPROM read failed", status);
    return false;
  }
#if THERMAL_DIAGNOSTIC_MODE
  Serial.println("[setup] EEPROM read succeeded");
#endif

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)
  {
    failSensorInitialization("Parameter extraction failed", status);
    return false;
  }
#if THERMAL_DIAGNOSTIC_MODE
  Serial.println("[setup] calibration parameters extracted");
#endif

  status = MLX90640_SetRefreshRate(MLX90640_address, REFRESH_RATE_SETTING);
  if (status != 0)
  {
    failSensorInitialization("Refresh rate setup failed", status);
    return false;
  }
  const int refreshRate = MLX90640_GetRefreshRate(MLX90640_address);
  if (refreshRate < 0)
  {
    failSensorInitialization("Refresh rate readback failed", refreshRate);
    return false;
  }
  if (refreshRate != REFRESH_RATE_SETTING)
  {
    failSensorInitialization("Refresh rate readback mismatch", refreshRate);
    return false;
  }

  return true;
}

void setup()
{
  Serial.begin(115200);
#if THERMAL_DIAGNOSTIC_MODE
  delay(1000);
  Serial.println("[setup] thermal diagnostic text mode");
  Serial.println("[setup] serial=115200 SDA=GPIO21 SCL=GPIO22 I2C=400000Hz address=0x33");
#endif

  int status = connectSensor();
  if (status != 0)
  {
    failSensorInitialization("MLX90640 not detected at 0x33", status);
    return;
  }
#if THERMAL_DIAGNOSTIC_MODE
  Serial.println("[setup] sensor detected");
#endif

  sensorReady = loadCalibrationAndSettings();
  if (!sensorReady)
  {
    return;
  }

#if THERMAL_DIAGNOSTIC_MODE
  Serial.println("[setup] refresh rate=8Hz");
  Serial.println("[setup] sensor ready; collecting subpages 0 and 1");
#endif
}

// MODIFIES: tempValues, using both halves of the sensor image.
// EFFECTS: Returns true only after subpages 0 and 1 have both arrived.
bool readTemperatureFrame()
{
  bool receivedSubpage[2] = {false, false};
  const byte maxAttempts = 6;

  for (byte attempt = 0; attempt < maxAttempts; ++attempt)
  {
    uint16_t frame[FRAME_WORD_COUNT];
    const int status = MLX90640_GetFrameData(MLX90640_address, frame);
    if (status < 0)
    {
      reportError("Frame read failed", status);
      continue;
    }
    if (status > 1)
    {
      reportError("Invalid subpage", status);
      continue;
    }

#if THERMAL_DIAGNOSTIC_MODE
    Serial.print("[frame] attempt=");
    Serial.print(attempt + 1);
    Serial.print(" subpage=");
    Serial.println(status);
#endif

    const float ambientTemperature = MLX90640_GetTa(frame, &mlx90640);
    MLX90640_CalculateTo(frame, &mlx90640, EMISSIVITY,
                         ambientTemperature - AMBIENT_SHIFT_C, tempValues);
    receivedSubpage[status] = true;
    if (receivedSubpage[0] && receivedSubpage[1])
    {
      return true;
    }
  }

  reportError("Incomplete frame discarded", -10);
  return false;
}

void sendTemperatureFrame()
{
  ++completedFrames;
#if THERMAL_DIAGNOSTIC_MODE
  reportTemperatureFrame();
#else
  const uint8_t header[] = {0xAA, 0xBB};
  Serial.write(header, sizeof(header));
  // Serial.write needs bytes; this cast exposes the existing float storage.
  Serial.write(reinterpret_cast<const uint8_t *>(tempValues), sizeof(tempValues));
#endif
}

void loop()
{
  if (!sensorReady)
  {
    if (sensorFailureMessage != nullptr)
      reportError(sensorFailureMessage, sensorFailureStatus);
    delay(100);
    return;
  }

  if (readTemperatureFrame())
  {
    sendTemperatureFrame();
  }
  delay(30);
}
