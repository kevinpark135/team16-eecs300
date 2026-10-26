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

namespace {
const unsigned int STATUS_REGISTER = 0x8000;

// The sensor sends each 16-bit word most-significant byte first.
void writeWord(uint16_t word) {
  Wire.write(static_cast<uint8_t>(word >> 8));
  Wire.write(static_cast<uint8_t>(word & 0xFF));
}
} // namespace

void MLX90640_I2CInit() {
  // The sketch configures Wire once, with the project's ESP32 pins.
}

int MLX90640_I2CRead(uint8_t deviceAddress, unsigned int startAddress,
                     unsigned int wordCount, uint16_t *data) {
  if (wordCount > 0 && data == nullptr) {
    return -1;
  }
  unsigned int wordsRead = 0;
  while (wordsRead < wordCount) {
    unsigned int chunkWords = wordCount - wordsRead;
    if (chunkWords > I2C_READ_CHUNK_BYTES / 2) {
      chunkWords = I2C_READ_CHUNK_BYTES / 2;
    }
    const unsigned int chunkBytes = chunkWords * 2;
    Wire.beginTransmission(deviceAddress);
    writeWord(static_cast<uint16_t>(startAddress));
    if (Wire.endTransmission(false) != 0) {
      return -1;
    }
    const size_t received = Wire.requestFrom(deviceAddress, static_cast<size_t>(chunkBytes));
    if (received != chunkBytes || Wire.available() < static_cast<int>(chunkBytes)) {
      return -3;
    }
    for (unsigned int i = 0; i < chunkWords; ++i) {
      const int highByte = Wire.read();
      const int lowByte = Wire.read();
      if (highByte < 0 || lowByte < 0) {
        return -3;
      }
      data[wordsRead] = static_cast<uint16_t>((highByte << 8) | lowByte);
      ++wordsRead;
    }
    startAddress += chunkWords;
  }
  return 0;
}

void MLX90640_I2CFreqSet(int frequencyKHz) {
  Wire.setClock(static_cast<unsigned long>(frequencyKHz) * 1000UL);
}

int MLX90640_I2CWrite(uint8_t deviceAddress, unsigned int writeAddress, uint16_t data) {
  Wire.beginTransmission(deviceAddress);
  writeWord(static_cast<uint16_t>(writeAddress));
  writeWord(data);
  if (Wire.endTransmission() != 0) {
    return -1;
  }
  uint16_t readback = 0;
  const int status = MLX90640_I2CRead(deviceAddress, writeAddress, 1, &readback);
  if (status != 0) {
    return status;
  }
#if THERMAL_DIAGNOSTIC_MODE
  static uint8_t statusWriteLogs = 0;
  if (writeAddress == STATUS_REGISTER && statusWriteLogs < 8) {
    Serial.print("[I2C write] address=0x");
    Serial.print(writeAddress, HEX);
    Serial.print(" wrote=0x");
    Serial.print(data, HEX);
    Serial.print(" read=0x");
    Serial.println(readback, HEX);
    ++statusWriteLogs;
  }
#endif
  // The sensor changes its status bits itself; only stable registers can match.
  if (writeAddress != STATUS_REGISTER && readback != data) {
    return -2;
  }
  return 0;
}
