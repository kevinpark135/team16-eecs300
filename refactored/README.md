# Sensor Code

This directory contains the integrated ESP32 counter, a standalone dual-LiDAR program, and the thermal firmware and desktop viewer. Shared headers and driver files are required by these programs.

## Integrated ESP32 Counter

| File | Role |
| --- | --- |
| [integrated.ino](integrated/integrated.ino) | Reads both LiDARs and the thermal camera on one ESP32, combines their results, and prints passage decisions and occupancy totals. |
| [passageCounter.hpp](integrated/passageCounter.hpp) | Applies the thermal-count decision rules, matches direction events within a time window, prevents repeated counts, and waits for the scene to clear. |
| [systemConfig.hpp](integrated/systemConfig.hpp) | Defines timing, thermal stability, and starting occupancy settings. |

## Dual LiDAR

| File | Role |
| --- | --- |
| [2LiDARs_direction.ino](lidar/2LiDARs_direction/2LiDARs_direction.ino) | Runs both LiDARs independently without thermal. Prints frame counts, detection status, and direction events. After successful sensor initialization and background calibration on both sensors, prints their latest 8x8 distance grids side by side every 500 ms in millimeters; X means an invalid measurement and -- means unavailable or stale data. Below the grids, RUNNING COUNT shows each sensor's cumulative IN and OUT events. IN is the configured positive (RIGHT) direction and OUT is the negative (LEFT) direction, matching the integrated counter. These are per-sensor events, not occupancy or a deduplicated people count. R resets background learning and local counts and hides the grids until both sensors finish calibration again. |
| [directionCounter.hpp](lidar/2LiDARs_direction/directionCounter.hpp) | Learns the background, follows an object's center along the selected axis, and emits a direction event once per observed object. |
| [directionSettings.hpp](lidar/2LiDARs_direction/directionSettings.hpp) | Selects the movement axis and direction reversal independently for L and R. |
| [lidarCommon.hpp](lidar/2LiDARs_direction/lidarCommon.hpp) | Defines pins and addresses, initializes the sensors, validates measurements, and provides optional startup logging. |

## Thermal Firmware

| File | Role |
| --- | --- |
| [MLX90640_Thermal_Camera.ino](thermal/MLX90640_Thermal_Camera/MLX90640_Thermal_Camera.ino) | Standalone ESP32 program that sends temperature frames to the desktop viewer using the AA BB + 3072-byte payload format. |
| [MLX90640_API.h](thermal/MLX90640_Thermal_Camera/MLX90640_API.h) | Defines calibration data and declares the manufacturer sensor API. |
| [MLX90640_API.cpp](thermal/MLX90640_Thermal_Camera/MLX90640_API.cpp) | Reads sensor frames and implements calibration extraction, sensor settings, and temperature calculations. |
| [MLX90640_I2C_Driver.h](thermal/MLX90640_Thermal_Camera/MLX90640_I2C_Driver.h) | Declares the I2C transport interface and read chunk size. |
| [MLX90640_I2C_Driver.cpp](thermal/MLX90640_Thermal_Camera/MLX90640_I2C_Driver.cpp) | Transfers sensor words through Arduino Wire and checks transfer/readback errors. |
| [thermalCamera.hpp](thermal/MLX90640_Thermal_Camera/thermalCamera.hpp) | Declares the integrated program's thermal adapter and its frame/calibration storage. |
| [thermalCamera.cpp](thermal/MLX90640_Thermal_Camera/thermalCamera.cpp) | Initializes thermal on the shared bus and collects complete subpage pairs across loop iterations. |
| [Thermal_Config.h](thermal/MLX90640_Thermal_Camera/Thermal_Config.h) | Selects binary or diagnostic-text output for the standalone thermal program. |

## Shared Detection and Desktop Viewer

| File | Role |
| --- | --- |
| [blobDetector.hpp](thermal/blobDetector.hpp) | Shared by the integrated ESP32 program and desktop viewer. Learns a thermal background and reports connected warm regions and data quality. |
| [thermaltest.cpp](thermal/thermaltest.cpp) | Opens the serial port and displays thermal frames and blob overlays using SDL. |
| [packetReader.hpp](thermal/packetReader.hpp) | Finds packet headers and reconstructs complete temperature payloads from serial bytes. |
| [floatPacket.hpp](thermal/floatPacket.hpp) | Defines the temperature packet and payload size. |

## Build Files

| File | Role |
| --- | --- |
| [Makefile](Makefile) | Builds the desktop viewer, starts firmware compilation, and removes generated output. |
| [thermal/Makefile](thermal/Makefile) | Compiles the SDL viewer into build/thermaltest. |
| [build_firmware.py](build_firmware.py) | Stages the required shared files into valid Arduino sketch folders under build/sketches and compiles the selected firmware. Does not upload to hardware. |
| [.gitignore](.gitignore) | Excludes regenerated build output from Git. |
