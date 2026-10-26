#include <Arduino.h>
#include "lidarCommon.hpp"
#include "systemConfig.hpp"
#include "thermalCamera.hpp"
#include "blobDetector.hpp"

SparkFun_VL53L5CX leftSensor;
SparkFun_VL53L5CX rightSensor;
VL53L5CX_ResultsData leftData;
VL53L5CX_ResultsData rightData;
direction_counter::DirectionCounter leftCounter(lidar::leftSettings());
direction_counter::DirectionCounter rightCounter(lidar::rightSettings());
ThermalCamera camera;
thermal::BlobDetector detector({}, thermal::Clock::time_point{});
const passage::CounterConfig counterConfig = settings::passageSettings();
passage::PassageCounter people(counterConfig);
bool ready = false;
int latestBlobCount = -1;
int lastThermalError = 0;
std::uint32_t lastThermalFrameAt = 0;
std::uint32_t lastThermalPoll = 0;
std::uint32_t lastStatusLog = 0;

passage::Motion passageMotion(direction_counter::Direction direction)
{
  if (direction == direction_counter::RIGHT)
  {
    return passage::ENTER;
  }
  if (direction == direction_counter::LEFT)
  {
    return passage::EXIT;
  }
  return passage::NONE;
}

void readLidar(passage::Side side, SparkFun_VL53L5CX &sensor,
               VL53L5CX_ResultsData &data,
               direction_counter::DirectionCounter &counter)
{
  if (!sensor.isDataReady() || !sensor.getRangingData(&data))
  {
    return;
  }
  int distances[lidar::ZONE_COUNT];
  bool valid[lidar::ZONE_COUNT];
  for (int zone = 0; zone < lidar::ZONE_COUNT; ++zone)
  {
    distances[zone] = data.distance_mm[zone];
    valid[zone] = lidar::validReading(data, zone);
  }
  const std::uint32_t now = millis();
  const direction_counter::Direction event = counter.processFrame(distances, valid, now);
  // A successful frame with no targets is still a fresh empty observation.
  if (!counter.isCalibrated())
  {
    return;
  }
  people.lidarFrame(side, counter.objectPresent(), passageMotion(event), now);
  if (event != direction_counter::NONE)
  {
    Serial.print(side == passage::LEFT_SENSOR ? "[L] " : "[R] ");
    Serial.println(event == direction_counter::RIGHT ? "ENTER candidate" : "EXIT candidate");
  }
}

void readThermal(std::uint32_t now)
{
  if (now - lastThermalPoll < settings::THERMAL_POLL_MS)
  {
    return;
  }
  lastThermalPoll = now;
  const int status = camera.poll(now);
  if (status == 0)
  {
    return;
  }
  now = millis();
  if (status < 0)
  {
    latestBlobCount = -1;
    lastThermalError = status;
    people.thermalFrame(0, false, now);
    return;
  }
  lastThermalFrameAt = now;
  lastThermalError = 0;
  // Only completed temperature frames reach the same detector used on the PC.
  const thermal::Clock::time_point timestamp{std::chrono::milliseconds(now)};
  const thermal::FrameResult &result = detector.processFrame(camera.temperatures(), timestamp);
  std::size_t count = 0;
  const bool valid = result.getBlobCount(count) && !result.dataDegraded;
  latestBlobCount = -1;
  if (valid)
  {
    latestBlobCount = static_cast<int>(count);
  }
  people.thermalFrame(latestBlobCount, valid, now);
}

void reportStatus(std::uint32_t now)
{
  if (now - lastStatusLog < settings::STATUS_LOG_MS)
  {
    return;
  }
  lastStatusLog = now;
  Serial.print("[people] thermal=");
  int displayedCount = latestBlobCount;
  if (now - lastThermalFrameAt > counterConfig.thermalFreshMs)
  {
    displayedCount = -1;
  }
  Serial.print(displayedCount); // -1 means unavailable, not an empty frame.
  Serial.print(" thermalError=");
  Serial.print(lastThermalError);
  Serial.print(" Lcal=");
  Serial.print(leftCounter.isCalibrated());
  Serial.print(" Rcal=");
  Serial.print(rightCounter.isCalibrated());
  Serial.print(" entries=");
  Serial.print(people.entries());
  Serial.print(" exits=");
  Serial.print(people.exits());
  Serial.print(" occupancy=");
  Serial.print(people.occupancy());
  Serial.print(" rejected=");
  Serial.print(people.rejected());
  Serial.print(" state=");
  Serial.print(static_cast<int>(people.state()));
  Serial.print(" freeHeap=");
  Serial.println(ESP.getFreeHeap());
}

void setup()
{
  Serial.begin(115200);
  delay(1500);
  Serial.println("Integrated L + thermal + R counter; mounting settings are unverified.");
  // All three sensors share Wire, so initialize it only once.
  if (!Wire.begin(lidar::SDA_PIN, lidar::SCL_PIN, settings::I2C_FREQUENCY_HZ))
  {
    Serial.println("I2C setup failed.");
    return;
  }
  Wire.setTimeOut(50);
  if (!lidar::startSensorPair(leftSensor, rightSensor, settings::I2C_FREQUENCY_HZ, false))
  {
    Serial.println("LiDAR setup failed.");
    return;
  }
  const int status = camera.begin();
  if (status != 0)
  {
    Serial.print("Thermal setup failed: ");
    Serial.println(status);
    return;
  }
  ready = true;
  Serial.println("Keep all views empty during background calibration.");
}

void reportDecision(passage::Decision decision)
{
  if (decision == passage::COUNTED)
  {
    Serial.println("[passage] counted");
  }
  if (decision == passage::REJECTED)
  {
    Serial.println("[passage] rejected");
  }
}

void loop()
{
  if (!ready)
  {
    delay(100);
    return;
  }
  reportDecision(people.update(millis()));
  readLidar(passage::LEFT_SENSOR, leftSensor, leftData, leftCounter);
  readLidar(passage::RIGHT_SENSOR, rightSensor, rightData, rightCounter);
  readThermal(millis());
  reportDecision(people.update(millis()));
  reportStatus(millis());
  delay(1);
}
