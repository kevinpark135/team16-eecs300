#pragma once

// Set to 1, upload, and use Serial Monitor at 115200 baud for readable sensor
// diagnostics. Set to 0 for the SDL viewer's binary AA BB + 3072-byte stream.
#ifndef THERMAL_DIAGNOSTIC_MODE
#define THERMAL_DIAGNOSTIC_MODE 0
#endif

#if THERMAL_DIAGNOSTIC_MODE != 0 && THERMAL_DIAGNOSTIC_MODE != 1
#error "THERMAL_DIAGNOSTIC_MODE must be 0 or 1"
#endif
