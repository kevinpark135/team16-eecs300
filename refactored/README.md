# Refactored Sensor Code

This directory contains an integrated ESP32 people counter, standalone sensor sketches, a desktop thermal viewer, and supporting tests and build tools.

## Integrated ESP32 Counter

The integrated sketch reads both LiDARs and the thermal camera on one ESP32. It uses thermal blob count to select between one-person direction agreement and two-person independent direction decisions.

| File | Role |
| --- | --- |
| [integrated.ino](integrated/integrated.ino) | Initializes the shared I2C bus and all sensors, runs independent L/R direction counters and thermal blob detection, feeds the passage counter, and prints decisions and totals. |
| [passageCounter.hpp](integrated/passageCounter.hpp) | Collects recent thermal evidence and direction events, applies each passage once, rejects stale or conflicting evidence, and waits for a clear scene before accepting another passage. |
| [systemConfig.hpp](integrated/systemConfig.hpp) | Defines matching, freshness, clear-scene timing, thermal stability, and initial occupancy settings. |
| [README.md](integrated/README.md) | Explains the algorithm, mounting settings, serial output, assumptions, build commands, and validation limits. |
| [passageCounter_test.cpp](integrated/tests/passageCounter_test.cpp) | Tests the decision table, delayed and repeated events, invalid/stale observations, thermal count changes, rearming, timer wrap, and synthetic sensor frames through the final count. |
| [frameReader_test.cpp](integrated/tests/frameReader_test.cpp) | Tests the manufacturer's new try-read entry point using simulated I2C functions, including pending frames, read failures, bounded retries, and legacy timeout behavior. |
| [thermalCamera_test.cpp](integrated/tests/thermalCamera_test.cpp) | Tests the thermal adapter using simulated sensor API calls, including initialization errors, paired subpages, missing halves, recovery, and timer wrap. |
| [Arduino.h](integrated/tests/fakes/Arduino.h) | Supplies minimal Arduino time declarations for desktop tests. |
| [Makefile](integrated/Makefile) | Builds and runs the passage and thermal adapter tests, including sanitizer checks. |

## LiDAR

The three `.ino` files are separate programs. Shared headers provide sensor setup and detection logic.

| File | Role |
| --- | --- |
| [2LiDARs.ino](lidar/2LiDARs.ino) | Reads two VL53L5CX sensors and prints their 8×8 distance grids side by side. |
| [direction_sensing.ino](lidar/direction_sensing.ino) | Reads one sensor, passes valid distance samples to the direction counter, and prints the grid and left/right totals. |
| [directionCounter.hpp](lidar/directionCounter.hpp) | Learns each sensor's background, follows the weighted center along a configurable row/column axis, and emits one confirmed direction event per observed object. Keeps local direction totals for debugging. |
| [2LiDARs_direction.ino](lidar/2LiDARs_direction/2LiDARs_direction.ino) | Runs a separate direction counter for each LiDAR and prints independent L/R ENTER/EXIT candidates. Does not calculate final occupancy without thermal evidence. |
| [directionSettings.hpp](lidar/directionSettings.hpp) | Defines the movement axis and direction reversal for each LiDAR, shared by the standalone dual-sensor test and integrated firmware. |
| [lidarCommon.hpp](lidar/lidarCommon.hpp) | Defines shared pins, addresses, and grid settings. Wraps SparkFun sensor initialization, reading validation, and distance formatting. |
| [counters_test.cpp](lidar/tests/counters_test.cpp) | Tests direction counting, background handling, noise rejection, timer behavior, axis selection, reversal, one-shot events, and independent L/R motion using simulated inputs. |
| [Makefile](lidar/Makefile) | Builds and runs the LiDAR logic tests, including sanitizer checks. |

## Thermal Camera Firmware

These files run on the ESP32 and use the included Melexis sensor API to produce temperature frames.

| File | Role |
| --- | --- |
| [MLX90640_Thermal_Camera.ino](thermal/MLX90640_Thermal_Camera/MLX90640_Thermal_Camera.ino) | Initializes the sensor, loads calibration, configures the refresh rate, collects both image subpages, and transmits complete frames. Binary output is `AA BB` followed by 768 floats (3072 payload bytes). |
| [MLX90640_API.h](thermal/MLX90640_Thermal_Camera/MLX90640_API.h) | Defines the calibration parameter structure and declares sensor configuration, frame-reading, try-read, and temperature-calculation functions. |
| [MLX90640_API.cpp](thermal/MLX90640_Thermal_Camera/MLX90640_API.cpp) | Implements the manufacturer sensor API, including EEPROM parameter extraction, register settings, ambient temperature estimation, and pixel temperature calculations. Provides both bounded waiting and immediate pending-result frame reads. |
| [MLX90640_I2C_Driver.h](thermal/MLX90640_Thermal_Camera/MLX90640_I2C_Driver.h) | Declares the I2C transport functions and defines the read chunk size. |
| [MLX90640_I2C_Driver.cpp](thermal/MLX90640_Thermal_Camera/MLX90640_I2C_Driver.cpp) | Uses Arduino `Wire` to read and write sensor words, convert between words and bytes, detect transfer failures, and check stable register writes. |
| [thermalCamera.hpp](thermal/MLX90640_Thermal_Camera/thermalCamera.hpp) | Declares the integrated firmware's thermal adapter and owns calibration data and the temperature buffer. |
| [thermalCamera.cpp](thermal/MLX90640_Thermal_Camera/thermalCamera.cpp) | Initializes the thermal sensor on the existing bus and collects subpages across loop iterations, discarding incomplete or failed pairs. |
| [Thermal_Config.h](thermal/MLX90640_Thermal_Camera/Thermal_Config.h) | Selects binary frame output or readable diagnostic output through `THERMAL_DIAGNOSTIC_MODE`. |

## Desktop Thermal Viewer

These files run on the computer. Serial bytes become complete temperature frames, which feed detection and display processing.

| File | Role |
| --- | --- |
| [floatPacket.hpp](thermal/floatPacket.hpp) | Defines the 768-float frame structure and payload size, and checks that floats are 32 bits. |
| [packetReader.hpp](thermal/packetReader.hpp) | Finds the `AA BB` header, collects the payload across partial reads, and reports when a complete frame is available. Also tracks parser counters. |
| [blobDetector.hpp](thermal/blobDetector.hpp) | Shared detector used by the PC viewer and integrated ESP32. Builds a median background, groups warmer pixels into blobs, and reports geometry, temperatures, calibration, invalid data, and stale results. |
| [thermaltest.cpp](thermal/thermaltest.cpp) | Implements `ThermalViewer`: opens the serial port, feeds bytes to the parser, runs detection on complete frames, draws the heatmap and blob overlays with SDL, prints diagnostics, and handles recalibration and resource cleanup. |
| [blobDetector_test.cpp](thermal/tests/blobDetector_test.cpp) | Tests detection geometry, thresholds, calibration, invalid samples, region selection, stale results, reset behavior, and packet parsing with synthetic frames. |
| [viewer_smoke.py](thermal/tests/viewer_smoke.py) | Runs the viewer with a pseudo-terminal and a headless SDL driver to check fragmented packets, calibration, detection, invalid data, stale results, and disconnection handling. |
| [Makefile](thermal/Makefile) | Builds the SDL viewer and runs the thermal tests and sanitizer checks. |
| [c_cpp_properties.json](thermal/.vscode/c_cpp_properties.json) | Configures VS Code C/C++ IntelliSense for the macOS compiler and SDL include paths. |

## Project Support

| File or Directory | Role |
| --- | --- |
| [Makefile](Makefile) | Provides shared targets for the desktop build, all test suites, sanitizer checks, the viewer smoke test, and firmware compilation. |
| [build_firmware.py](build_firmware.py) | Stages each LiDAR sketch in its own Arduino sketch directory with shared headers. Compiles the integrated program, all three LiDAR programs, and both standalone thermal output modes using installed Arduino tools, without uploading firmware. |
| [.gitignore](.gitignore) | Excludes the generated project-level `build/` directory from Git. |
| `build/` | Holds generated executables, staged sketches, firmware binaries, local Arduino build state, caches, and validation logs. |
