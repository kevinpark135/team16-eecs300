# Refactored Sensor Code

This directory contains three LiDAR sketches, thermal camera firmware, a desktop thermal viewer, and supporting tests and build tools.

## LiDAR

The three `.ino` files are separate programs. Shared headers provide sensor setup and detection logic.

| File | Role |
| --- | --- |
| [2LiDARs.ino](lidar/2LiDARs.ino) | Reads two VL53L5CX sensors and prints their 8×8 distance grids side by side. |
| [direction_sensing.ino](lidar/direction_sensing.ino) | Reads one sensor, passes valid distance samples to the direction counter, and prints the grid and left/right totals. |
| [directionCounter.hpp](lidar/directionCounter.hpp) | Learns the background, finds an object's weighted horizontal center, estimates motion from recent positions, and counts a confirmed direction once per tracked object. |
| [2LiDARs_direction.ino](lidar/2LiDARs_direction/2LiDARs_direction.ino) | Splits two sensors' grids into left and right lanes, checks nearby targets, and prints entering or leaving events for each lane. |
| [laneCounter.hpp](lidar/laneCounter.hpp) | Tracks which sensor activates first in each lane and returns a crossing event when the other sensor activates. Resets pending state on timeout. |
| [lidarCommon.hpp](lidar/lidarCommon.hpp) | Defines shared pins, addresses, and grid settings. Wraps SparkFun sensor initialization, reading validation, and distance formatting. |
| [counters_test.cpp](lidar/tests/counters_test.cpp) | Tests direction counting, background handling, noise rejection, timer behavior, lane independence, and crossing order using simulated inputs. |
| [Makefile](lidar/Makefile) | Builds and runs the LiDAR logic tests, including sanitizer checks. |

## Thermal Camera Firmware

These files run on the ESP32 and use the included Melexis sensor API to produce temperature frames.

| File | Role |
| --- | --- |
| [MLX90640_Thermal_Camera.ino](thermal/MLX90640_Thermal_Camera/MLX90640_Thermal_Camera.ino) | Initializes the sensor, loads calibration, configures the refresh rate, collects both image subpages, and transmits complete frames. Binary output is `AA BB` followed by 768 floats (3072 payload bytes). |
| [MLX90640_API.h](thermal/MLX90640_Thermal_Camera/MLX90640_API.h) | Defines the calibration parameter structure and declares sensor configuration, frame-reading, and temperature-calculation functions. |
| [MLX90640_API.cpp](thermal/MLX90640_Thermal_Camera/MLX90640_API.cpp) | Implements the manufacturer sensor API, including EEPROM parameter extraction, register settings, ambient temperature estimation, and pixel temperature calculations. |
| [MLX90640_I2C_Driver.h](thermal/MLX90640_Thermal_Camera/MLX90640_I2C_Driver.h) | Declares the I2C transport functions and defines the read chunk size. |
| [MLX90640_I2C_Driver.cpp](thermal/MLX90640_Thermal_Camera/MLX90640_I2C_Driver.cpp) | Uses Arduino `Wire` to read and write sensor words, convert between words and bytes, detect transfer failures, and check stable register writes. |
| [Thermal_Config.h](thermal/MLX90640_Thermal_Camera/Thermal_Config.h) | Selects binary frame output or readable diagnostic output through `THERMAL_DIAGNOSTIC_MODE`. |

## Desktop Thermal Viewer

These files run on the computer. Serial bytes become complete temperature frames, which feed detection and display processing.

| File | Role |
| --- | --- |
| [floatPacket.hpp](thermal/floatPacket.hpp) | Defines the 768-float frame structure and payload size, and checks that floats are 32 bits. |
| [packetReader.hpp](thermal/packetReader.hpp) | Finds the `AA BB` header, collects the payload across partial reads, and reports when a complete frame is available. Also tracks parser counters. |
| [blobDetector.hpp](thermal/blobDetector.hpp) | Builds a median background, identifies warmer pixels, groups connected pixels into blobs, and calculates their area, center, bounding box, and temperatures. Tracks calibration, invalid data, and stale results. |
| [thermaltest.cpp](thermal/thermaltest.cpp) | Implements `ThermalViewer`: opens the serial port, feeds bytes to the parser, runs detection on complete frames, draws the heatmap and blob overlays with SDL, prints diagnostics, and handles recalibration and resource cleanup. |
| [blobDetector_test.cpp](thermal/tests/blobDetector_test.cpp) | Tests detection geometry, thresholds, calibration, invalid samples, region selection, stale results, reset behavior, and packet parsing with synthetic frames. |
| [viewer_smoke.py](thermal/tests/viewer_smoke.py) | Runs the viewer with a pseudo-terminal and a headless SDL driver to check fragmented packets, calibration, detection, invalid data, stale results, and disconnection handling. |
| [Makefile](thermal/Makefile) | Builds the SDL viewer and runs the thermal tests and sanitizer checks. |
| [c_cpp_properties.json](thermal/.vscode/c_cpp_properties.json) | Configures VS Code C/C++ IntelliSense for the macOS compiler and SDL include paths. |

## Project Support

| File or Directory | Role |
| --- | --- |
| [Makefile](Makefile) | Provides shared targets for the desktop build, both test suites, sanitizer checks, the viewer smoke test, and firmware compilation. |
| [build_firmware.py](build_firmware.py) | Stages each LiDAR sketch in its own Arduino sketch directory with shared headers. Compiles all three LiDAR programs and both thermal output modes using installed Arduino tools, without uploading firmware. |
| [.gitignore](.gitignore) | Excludes the generated project-level `build/` directory from Git. |
| `build/` | Holds generated executables, staged sketches, firmware binaries, local Arduino build state, caches, and validation logs. |
