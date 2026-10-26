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
#pragma once

#include <stdint.h>

// A small, even chunk fits the ESP32 Wire receive buffer.
const unsigned int I2C_READ_CHUNK_BYTES = 32;

void MLX90640_I2CInit();
// REQUIRES: data has room for wordCount words; addresses are sensor word addresses.
// MODIFIES: data and Wire.
// EFFECTS: Returns 0 on success, -1 on a bus error, or -3 on a short read.
int MLX90640_I2CRead(uint8_t deviceAddress, unsigned int startAddress,
                     unsigned int wordCount, uint16_t *data);
// EFFECTS: Returns 0 on success, or -1/-2/-3 for bus/readback/read errors.
int MLX90640_I2CWrite(uint8_t deviceAddress, unsigned int writeAddress, uint16_t data);
void MLX90640_I2CFreqSet(int frequencyKHz);
