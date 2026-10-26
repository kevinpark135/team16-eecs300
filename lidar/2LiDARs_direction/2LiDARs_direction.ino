#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> 

// Define unique LPn (Shutdown) control pins
#define LPN_PIN_1 26
#define LPN_PIN_2 19

// Create independent driver instances
SparkFun_VL53L5CX sensor1;
SparkFun_VL53L5CX sensor2;

// Ranging data structures
VL53L5CX_ResultsData data1;
VL53L5CX_ResultsData data2;

const int PERSON_THRESHOLD_MM = 150;
const unsigned long TIMEOUT_MS = 500; // Max time between sensors

// Lane 1 State (e.g., Left side of doorway)
enum DirectionState { IDLE, S1_FIRST, S2_FIRST };
DirectionState lane1State = IDLE;
unsigned long lane1Timer = 0;

//lane 2:
DirectionState lane2State = IDLE;
unsigned long lane2Timer = 0;

void setup() {
  Serial.begin(115200);
  delay(1500); 

  // Set up LPn pins as outputs
  pinMode(LPN_PIN_1, OUTPUT);
  pinMode(LPN_PIN_2, OUTPUT);

  // Hard reset both sensors by pulling LPn LOW
  Serial.println("Step 1: Pulling LPn low to reset sensors...");
  digitalWrite(LPN_PIN_1, LOW);
  digitalWrite(LPN_PIN_2, LOW);
  delay(200);

  // Initialize Standard I2C Bus on ESP32
  Serial.println("Step 2: Starting I2C Bus...");
  Wire.begin(21, 22); 
  Wire.setClock(400000); 
  
  Serial.println("Step 3: Attempting Sensor 1 Wakeup...");
  digitalWrite(LPN_PIN_1, HIGH);
  delay(200); 

  Serial.println("Step 4: Calling sensor1.begin()...");
  if (!sensor1.begin()) {
    Serial.println("ERROR: Sensor 1 failed to reply at default 0x29! Check I2C lines.");
    while (1);
  }
  
  Serial.println("Step 5: Changing Sensor 1 Address...");
  if (!sensor1.setAddress(0x30)) {
    Serial.println("ERROR: Failed to assign 0x30 to Sensor 1!");
    while (1);
  }
  Serial.println("Sensor 1 mapped to 0x30 successfully.");

  Serial.println("Step 6: Attempting Sensor 2 Wakeup...");
  digitalWrite(LPN_PIN_2, HIGH); 
  delay(200);

  Serial.println("Step 7: Calling sensor2.begin()...");
  if (!sensor2.begin()) {
    Serial.println("ERROR: Sensor 2 failed to reply at default 0x29!");
    while (1);
  }
  
  if (!sensor2.setAddress(0x32)) {
    Serial.println("ERROR: Failed to assign 0x32 to Sensor 2!");
    while (1);
  }
  Serial.println("Sensor 2 mapped to 0x32 successfully.");

  // Remaining configuration...
  sensor1.setResolution(8 * 8); 
  sensor2.setResolution(8 * 8); 
  sensor1.setRangingFrequency(15);
  sensor2.setRangingFrequency(15);
  sensor1.startRanging();
  sensor2.startRanging();
}

void loop() {
  // Poll both sensors until fresh frames are ready
  if (sensor1.isDataReady() && sensor2.isDataReady()) {
    sensor1.getRangingData(&data1);
    sensor2.getRangingData(&data2);

    // Frame separator line
    Serial.println("");
    Serial.println("      --- SENSOR 1 (0x30) ---            |            --- SENSOR 2 (0x32) ---      ");
    Serial.println("");

    unsigned long currentMillis = millis();

    // --- LANE 1 (Columns 0 to 3 - Left side) ---
    int min1_L = getMinDistanceColumns(data1, 0, 3);
    int min2_L = getMinDistanceColumns(data2, 0, 3);
    bool s1_L_active = (min1_L > 0 && min1_L < PERSON_THRESHOLD_MM);
    bool s2_L_active = (min2_L > 0 && min2_L < PERSON_THRESHOLD_MM);
    processLane("Lane 1 (Left)", lane1State, lane1Timer, s1_L_active, s2_L_active, currentMillis);

    // --- LANE 2 (Columns 4 to 7 - Right side) ---
    int min1_R = getMinDistanceColumns(data1, 4, 7);
    int min2_R = getMinDistanceColumns(data2, 4, 7);
    bool s1_R_active = (min1_R > 0 && min1_R < PERSON_THRESHOLD_MM);
    bool s2_R_active = (min2_R > 0 && min2_R < PERSON_THRESHOLD_MM);
    processLane("Lane 2 (Right)", lane2State, lane2Timer, s1_R_active, s2_R_active, currentMillis);
  }
}
// Helper to check min distance within a specific range of columns (0-7)
int getMinDistanceColumns(VL53L5CX_ResultsData &data, int startCol, int endCol) {
  int min_dist = 9999;
  
  for (int row = 0; row < 8; row++) {
    for (int col = startCol; col <= endCol; col++) {
      int index = (row * 8) + col;
      int dist = data.distance_mm[index];
      if (dist > 0 && dist < min_dist) {
        min_dist = dist;
      }
    }
  }
  return (min_dist == 9999) ? 0 : min_dist;
}void processLane(const char* laneName, DirectionState &state, unsigned long &timer, bool s1_active, bool s2_active, unsigned long currentMillis) {
  switch (state) {
    case IDLE:
      if (s1_active && !s2_active) {
        state = S1_FIRST;
        timer = currentMillis;
      } else if (s2_active && !s1_active) {
        state = S2_FIRST;
        timer = currentMillis;
      }
      break;

    case S1_FIRST:
      if (s2_active) {
        Serial.print(laneName);
        Serial.println(": ===> ENTERING (Sensor 1 -> Sensor 2)");
        state = IDLE;
      } else if (currentMillis - timer > TIMEOUT_MS) {
        state = IDLE;
      }
      break;

    case S2_FIRST:
      if (s1_active) {
        Serial.print(laneName);
        Serial.println(": <=== LEAVING (Sensor 2 -> Sensor 1)");
        state = IDLE;
      } else if (currentMillis - timer > TIMEOUT_MS) {
        state = IDLE;
      }
      break;
  }
}
// Utility to ensure grid elements keep uniform width formatting in Serial Monitor
void printFormattedDistance(int distance) {
  if (distance < 10) Serial.print("   ");
  else if (distance < 100) Serial.print("  ");
  else if (distance < 1000) Serial.print(" ");
  
  Serial.print(distance);
  Serial.print(" ");
}
