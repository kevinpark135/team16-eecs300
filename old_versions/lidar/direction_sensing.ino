#include <Wire.h>
#include <math.h>
#include <SparkFun_VL53L5CX_Library.h>

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData;


// ============================================================
// SETTINGS
// ============================================================

const int GRID_SIZE = 8;
const int NUM_ZONES = 64;

// Sensor runs at 15 Hz.
// We print every 3 frames -> Serial Monitor updates ~5 Hz.
const int DISPLAY_DIVIDER = 1;

// Short startup calibration used only to learn fixed objects.
const int CALIBRATION_FRAMES = 15;

// If a background distance exists, something must become
// this much closer to count as a moving object.
const int OBJECT_DELTA_MM = 150;

// If there was NO background target in a zone,
// a new target within this distance can be an object.
const int MAX_OBJECT_DISTANCE_MM = 1200;

// Require multiple cells so random noise is ignored.
const int MIN_OBJECT_CELLS = 3;

// Recent motion history.
// 6 frames at 15 Hz = about 0.4 seconds.
const int HISTORY_SIZE = 6;

// Horizontal speed threshold in grid columns / second.
const float MIN_HORIZONTAL_SPEED = 1.5;

// Object should actually move horizontally by this much.
const float MIN_HORIZONTAL_TRAVEL = 1.0;

// Require the same direction for multiple consecutive estimates.
const int DIRECTION_CONFIRM_FRAMES = 3;

// Object must disappear before it can be counted again.
const int LOST_RESET_FRAMES = 5;


// ============================================================
// BACKGROUND DATA
// ============================================================

uint32_t baselineSum[NUM_ZONES] = {0};
uint16_t baselineSamples[NUM_ZONES] = {0};

float baseline[NUM_ZONES] = {0};
bool baselineValid[NUM_ZONES] = {false};

int calibrationFrame = 0;
bool calibrated = false;


// ============================================================
// MOTION HISTORY
// ============================================================

float xHistory[HISTORY_SIZE];
float tHistory[HISTORY_SIZE];

int historyCount = 0;


// ============================================================
// DIRECTION COUNTS
// ============================================================

enum Direction
{
  DIR_NONE,
  DIR_LEFT,
  DIR_RIGHT
};

Direction candidateDirection = DIR_NONE;

int directionStreak = 0;
int lostFrames = 0;

bool countedCurrentObject = false;

unsigned long leftCount = 0;
unsigned long rightCount = 0;

unsigned long frameNumber = 0;


// ============================================================
// VALID READING
// ============================================================

bool validReading(int zone)
{
  int status = measurementData.target_status[zone];

  return (
    measurementData.nb_target_detected[zone] > 0 &&
    (status == 5 || status == 9) &&
    measurementData.distance_mm[zone] > 0
  );
}


// ============================================================
// PRINT 8x8 MATRIX
// ============================================================

void printMatrix()
{
  for (int row = 0; row < 8; row++)
  {
    for (int col = 0; col < 8; col++)
    {
      int zone = row * 8 + col;

      if (validReading(zone))
      {
        Serial.print(measurementData.distance_mm[zone]);
      }
      else
      {
        Serial.print("X");
      }

      Serial.print("\t");
    }

    Serial.println();
  }
}


// ============================================================
// ADD CENTROID POSITION TO HISTORY
// ============================================================

void addHistory(float x)
{
  float currentTime = millis() / 1000.0f;

  if (historyCount < HISTORY_SIZE)
  {
    xHistory[historyCount] = x;
    tHistory[historyCount] = currentTime;

    historyCount++;
  }
  else
  {
    for (int i = 0; i < HISTORY_SIZE - 1; i++)
    {
      xHistory[i] = xHistory[i + 1];
      tHistory[i] = tHistory[i + 1];
    }

    xHistory[HISTORY_SIZE - 1] = x;
    tHistory[HISTORY_SIZE - 1] = currentTime;
  }
}


// ============================================================
// LINEAR REGRESSION
// Returns horizontal speed in grid columns / second
// ============================================================

float calculateHorizontalSpeed()
{
  float meanT = 0;
  float meanX = 0;

  for (int i = 0; i < HISTORY_SIZE; i++)
  {
    meanT += tHistory[i];
    meanX += xHistory[i];
  }

  meanT /= HISTORY_SIZE;
  meanX /= HISTORY_SIZE;

  float numerator = 0;
  float denominator = 0;

  for (int i = 0; i < HISTORY_SIZE; i++)
  {
    float dt = tHistory[i] - meanT;
    float dx = xHistory[i] - meanX;

    numerator += dt * dx;
    denominator += dt * dt;
  }

  if (denominator == 0)
    return 0;

  return numerator / denominator;
}


// ============================================================
// RESET FOR NEXT OBJECT
// ============================================================

void resetTracking()
{
  historyCount = 0;

  candidateDirection = DIR_NONE;
  directionStreak = 0;

  countedCurrentObject = false;
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("VL53L7CX Direction Counter");

  // ESP32 I2C
  // SDA = GPIO 21
  // SCL = GPIO 22
  Wire.begin(21, 22);
  Wire.setClock(400000);

  Serial.println("Initializing sensor...");

  if (myImager.begin() == false)
  {
    Serial.println("Sensor not found.");
    while (1);
  }

  // 8x8 mode
  myImager.setResolution(64);

  // Sensor measures internally at 15 Hz.
  myImager.setRangingFrequency(15);

  myImager.startRanging();

  Serial.println("Sensor ready.");
  Serial.println("Keep scene still briefly...");
  Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  if (myImager.isDataReady() == false)
    return;

  if (myImager.getRangingData(&measurementData) == false)
    return;

  frameNumber++;


  // ==========================================================
  // STARTUP BACKGROUND LEARNING
  // ==========================================================

  if (!calibrated)
  {
    for (int zone = 0; zone < NUM_ZONES; zone++)
    {
      if (validReading(zone))
      {
        baselineSum[zone] += measurementData.distance_mm[zone];
        baselineSamples[zone]++;
      }
    }

    calibrationFrame++;


    // Only update Serial Monitor ~5 times/sec
    if (frameNumber % DISPLAY_DIVIDER == 0)
    {
      Serial.print("Frame ");
      Serial.println(frameNumber);

      printMatrix();

      Serial.println();

      Serial.print("LEFT: ");
      Serial.print(leftCount);

      Serial.print("          RIGHT: ");
      Serial.println(rightCount);

      Serial.println();
      Serial.println("----------------------------------------");
    }


    if (calibrationFrame >= CALIBRATION_FRAMES)
    {
      for (int zone = 0; zone < NUM_ZONES; zone++)
      {
        // Treat it as fixed background only if that target
        // appeared consistently during calibration.
        if (baselineSamples[zone] >= CALIBRATION_FRAMES / 2)
        {
          baselineValid[zone] = true;

          baseline[zone] =
            (float)baselineSum[zone] /
            baselineSamples[zone];
        }
        else
        {
          baselineValid[zone] = false;
          baseline[zone] = 0;
        }
      }

      calibrated = true;

      Serial.println();
      Serial.println("READY");
      Serial.println();
    }

    return;
  }


  // ==========================================================
  // DETECT MOVING OBJECT CELLS
  // ==========================================================

  int objectCells = 0;

  float weightedX = 0;
  float totalWeight = 0;


  for (int row = 0; row < 8; row++)
  {
    for (int col = 0; col < 8; col++)
    {
      int zone = row * 8 + col;

      if (!validReading(zone))
        continue;

      float distance = measurementData.distance_mm[zone];

      bool objectDetected = false;


      // -------------------------------------------------------
      // Case 1:
      // This zone had a fixed background target.
      //
      // Current target must be significantly closer.
      // -------------------------------------------------------

      if (baselineValid[zone])
      {
        float difference =
          baseline[zone] - distance;

        if (difference >= OBJECT_DELTA_MM)
        {
          objectDetected = true;
        }
      }


      // -------------------------------------------------------
      // Case 2:
      // This zone was originally empty (X).
      //
      // A new valid target appearing nearby is an object.
      // -------------------------------------------------------

      else
      {
        if (distance <= MAX_OBJECT_DISTANCE_MM)
        {
          objectDetected = true;
        }
      }


      if (objectDetected)
      {
        objectCells++;

        // Closer cells receive slightly more influence.
        float weight =
          1000.0f / (distance + 1.0f);

        weightedX += col * weight;
        totalWeight += weight;
      }
    }
  }


  // ==========================================================
  // OBJECT PRESENT
  // ==========================================================

  if (
    objectCells >= MIN_OBJECT_CELLS &&
    totalWeight > 0
  )
  {
    lostFrames = 0;

    float centroidX =
      weightedX / totalWeight;

    addHistory(centroidX);


    // Need six positions before estimating direction.
    if (historyCount == HISTORY_SIZE)
    {
      float horizontalSpeed =
        calculateHorizontalSpeed();

      float minX = xHistory[0];
      float maxX = xHistory[0];

      for (int i = 1; i < HISTORY_SIZE; i++)
      {
        if (xHistory[i] < minX)
          minX = xHistory[i];

        if (xHistory[i] > maxX)
          maxX = xHistory[i];
      }

      float horizontalTravel = maxX - minX;


      Direction newDirection = DIR_NONE;


      if (
        horizontalTravel >= MIN_HORIZONTAL_TRAVEL &&
        fabs(horizontalSpeed) >= MIN_HORIZONTAL_SPEED
      )
      {
        // You already verified that increasing columns
        // correspond to physical movement toward RIGHT.

        if (horizontalSpeed > 0)
        {
          newDirection = DIR_RIGHT;
        }
        else
        {
          newDirection = DIR_LEFT;
        }
      }


      // ======================================================
      // STABILIZE DIRECTION ESTIMATE
      // ======================================================

      if (newDirection == DIR_NONE)
      {
        candidateDirection = DIR_NONE;
        directionStreak = 0;
      }

      else if (newDirection == candidateDirection)
      {
        directionStreak++;
      }

      else
      {
        candidateDirection = newDirection;
        directionStreak = 1;
      }


      // ======================================================
      // COUNT OBJECT ONCE
      // ======================================================

      if (
        directionStreak >= DIRECTION_CONFIRM_FRAMES &&
        !countedCurrentObject
      )
      {
        if (candidateDirection == DIR_LEFT)
        {
          leftCount++;
        }

        else if (candidateDirection == DIR_RIGHT)
        {
          rightCount++;
        }

        countedCurrentObject = true;
      }
    }
  }


  // ==========================================================
  // NO OBJECT
  // ==========================================================

  else
  {
    lostFrames++;

    if (lostFrames >= LOST_RESET_FRAMES)
    {
      resetTracking();
      lostFrames = 0;
    }
  }


  // ==========================================================
  // DISPLAY ~5 Hz
  // ==========================================================

  if (frameNumber % DISPLAY_DIVIDER == 0)
  {
    Serial.print("Frame ");
    Serial.println(frameNumber);

    printMatrix();

    Serial.println();

    Serial.print("LEFT: ");
    Serial.print(leftCount);

    Serial.print("          RIGHT: ");
    Serial.println(rightCount);

    Serial.println();
    Serial.println("----------------------------------------");
  }
}