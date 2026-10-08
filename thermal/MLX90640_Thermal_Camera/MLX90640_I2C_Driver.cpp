/**
   @copyright (C) 2017 Melexis N.V.
   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at
       http://www.apache.org/licenses/LICENSE-2.0
   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#include <Arduino.h>

#include <Wire.h>

#include "MLX90640_I2C_Driver.h"
#include "Thermal_Config.h"

void MLX90640_I2CInit()
{
}

// Read a number of words from startAddress. Store into Data array.
// Returns 0 only on a complete read; negative values indicate errors.
int MLX90640_I2CRead(uint8_t _deviceAddress, unsigned int startAddress, unsigned int nWordsRead, uint16_t *data)
{

  // Caller passes number of 'unsigned ints to read', increase this to 'bytes to read'
  uint16_t bytesRemaining = nWordsRead * 2;

  // It doesn't look like sequential read works. Do we need to re-issue the address command each time?

  uint16_t dataSpot = 0; // Start at beginning of array

  // Setup a series of chunked I2C_BUFFER_LENGTH byte reads
  while (bytesRemaining > 0)
  {
    Wire.beginTransmission(_deviceAddress);
    Wire.write(startAddress >> 8);        // MSB
    Wire.write(startAddress & 0xFF);      // LSB
    if (Wire.endTransmission(false) != 0) // Do not release bus
    {
      return -1; // Sensor did not ACK
    }

    uint16_t numberOfBytesToRead = bytesRemaining;
    if (numberOfBytesToRead > I2C_BUFFER_LENGTH)
      numberOfBytesToRead = I2C_BUFFER_LENGTH;

    size_t received = Wire.requestFrom((uint8_t)_deviceAddress, (size_t)numberOfBytesToRead);
    if (received != numberOfBytesToRead || Wire.available() < numberOfBytesToRead)
    {
      return -3; // Short read: never consume missing bytes.
    }
    for (uint16_t x = 0; x < numberOfBytesToRead / 2; x++)
    {
      int msb = Wire.read();
      int lsb = Wire.read();
      if (msb < 0 || lsb < 0)
        return -3;
      data[dataSpot++] = (static_cast<uint16_t>(msb) << 8) | lsb;
    }

    bytesRemaining -= numberOfBytesToRead;

    startAddress += numberOfBytesToRead / 2;
  }

  return (0); // Success
}

// Set I2C Freq, in kHz
// MLX90640_I2CFreqSet(1000) sets frequency to 1MHz
void MLX90640_I2CFreqSet(int freq)
{
  // i2c.frequency(1000 * freq);
  Wire.setClock((long)1000 * freq);
}

// Write two bytes to a two byte address
int MLX90640_I2CWrite(uint8_t _deviceAddress, unsigned int writeAddress, uint16_t data)
{
  Wire.beginTransmission((uint8_t)_deviceAddress);
  Wire.write(writeAddress >> 8);   // MSB
  Wire.write(writeAddress & 0xFF); // LSB
  Wire.write(data >> 8);           // MSB
  Wire.write(data & 0xFF);         // LSB
  if (Wire.endTransmission() != 0)
  {
    // Sensor did not ACK
    return (-1);
  }

  uint16_t dataCheck;
  int error = MLX90640_I2CRead(_deviceAddress, writeAddress, 1, &dataCheck);
  if (error != 0)
    return error;

#if THERMAL_DIAGNOSTIC_MODE
  if (writeAddress == 0x8000)
  {
    static uint8_t statusWriteLogs = 0;
    if (statusWriteLogs < 8)
    {
      Serial.print("[I2C write] address=0x");
      Serial.print(writeAddress, HEX);
      Serial.print(" wrote=0x");
      Serial.print(data, HEX);
      Serial.print(" read=0x");
      Serial.println(dataCheck, HEX);
      ++statusWriteLogs;
    }
  }
#endif

  // 0x8000 is a dynamic status register. Bits include the last measured
  // subpage and the new-data flag, so equality after writing 0x0030 is not a
  // valid write check. Keep the read itself to detect ACK and short-read
  // failures, but only compare stable configuration registers.
  if (writeAddress != 0x8000 && dataCheck != data)
  {
    return -2;
  }

  return (0); // Success
}
