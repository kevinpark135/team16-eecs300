#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> // Works with VL53L7CX seamlessly

// Define unique LPn (Shutdown) control pins
#define LPN_PIN_1 4
#define LPN_PIN_2 5

// Create independent driver instances
SparkFun_VL53L5CX sensor1;
SparkFun_VL53L5CX sensor2;

// Ranging data structures
VL53L5CX_ResultsData data1;
VL53L5CX_ResultsData data2;

void setup() {
  Serial.begin(115200);
  
  // Give the computer a moment to attach the port
  delay(1500); 
  
  Serial.println("\n=========================================");
  Serial.println("MCU ALIVE: Serial communication started!");
  Serial.println("=========================================");

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
  Wire.setClock(100000); // LOWERED to 100kHz for initial troubleshooting stableness
  
  Serial.println("Step 3: Attempting Sensor 1 Wakeup...");
  digitalWrite(LPN_PIN_1, HIGH); // Wake up Sensor 1
  delay(200); 

  Serial.println("Step 4: Calling sensor1.begin()...");
  if (!sensor1.begin()) {
    Serial.println(">>> CRITICAL ERROR: Sensor 1 failed to reply at default 0x29! Check I2C lines.");
    while (1);
  }
  
  Serial.println("Step 5: Changing Sensor 1 Address...");
  if (!sensor1.setAddress(0x30)) {
    Serial.println(">>> CRITICAL ERROR: Failed to assign 0x30 to Sensor 1!");
    while (1);
  }
  Serial.println("Sensor 1 mapped to 0x30 successfully.");

  Serial.println("Step 6: Attempting Sensor 2 Wakeup...");
  digitalWrite(LPN_PIN_2, HIGH); 
  delay(200);

  Serial.println("Step 7: Calling sensor2.begin()...");
  if (!sensor2.begin()) {
    Serial.println(">>> CRITICAL ERROR: Sensor 2 failed to reply at default 0x29!");
    while (1);
  }
  
  if (!sensor2.setAddress(0x32)) {
    Serial.println(">>> CRITICAL ERROR: Failed to assign 0x32 to Sensor 2!");
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
  
  Serial.println("\n>>> SUCCESS: Setup complete. Starting stream!");
}

void loop() {
  // Poll both sensors until fresh frames are ready
  if (sensor1.isDataReady() && sensor2.isDataReady()) {
    sensor1.getRangingData(&data1);
    sensor2.getRangingData(&data2);

    // Frame separator line
    Serial.println("=================================================================================");
    Serial.println("      --- SENSOR 1 (0x30) ---            |            --- SENSOR 2 (0x32) ---      ");
    Serial.println("=================================================================================");

    // Print the two grids side-by-side
    for (int row = 0; row < 8; row++) {
      
      // Print Sensor 1 Row (Note: VL53L7CX stores row blocks inverted or straight depending on your orientation)
      for (int col = 0; col < 8; col++) {
        int index = (row * 8) + col;
        printFormattedDistance(data1.distance_mm[index]);
      }

      // Visual divider boundary between sensors
      Serial.print(" |  ");

      // Print Sensor 2 Row
      for (int col = 0; col < 8; col++) {
        int index = (row * 8) + col;
        printFormattedDistance(data2.distance_mm[index]);
      }
      
      Serial.println(); // Next line
    }
    Serial.println(); 
    delay(50); // Minor stabilization delay
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
