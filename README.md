# Sensor Code

## Integrated Counter

| File | Role |
| --- | --- |
| [integrated.ino](integrated/integrated.ino) | Combines LiDAR direction events and thermal blob counts to track passages and occupancy. |
| [passageCounter.hpp](integrated/passageCounter.hpp) | Matches sensor events and updates entry, exit, and occupancy counts. |
| [systemConfig.hpp](integrated/systemConfig.hpp) | Defines timing and counting settings. |

## Single LiDAR

| File | Role |
| --- | --- |
| [direction_sensing.ino](lidar/direction_sensing/direction_sensing.ino) | Reads one LiDAR and prints its 8x8 distance grid and LEFT/RIGHT counts. |

## Dual LiDAR

| File | Role |
| --- | --- |
| [2LiDARs_direction.ino](lidar/2LiDARs_direction/2LiDARs_direction.ino) | Reads two LiDARs and prints side-by-side 8x8 distance grids and IN/OUT counts. |
| [directionCounter.hpp](lidar/2LiDARs_direction/directionCounter.hpp) | Learns the background and confirms consistent movement from recent positions, counting each observed object once. |
| [directionSettings.hpp](lidar/2LiDARs_direction/directionSettings.hpp) | Sets the movement axis and direction reversal for each LiDAR. |
| [lidarCommon.hpp](lidar/2LiDARs_direction/lidarCommon.hpp) | Defines hardware settings, initializes both LiDARs, and validates readings. |
| [directionCounter_test.cpp](lidar/2LiDARs_direction/tests/directionCounter_test.cpp) | Checks fast passages, mounting direction, noise rejection, and duplicate-count prevention using synthetic frames. |

## Thermal Firmware

| File | Role |
| --- | --- |
| [MLX90640_Thermal_Camera.ino](thermal/MLX90640_Thermal_Camera/MLX90640_Thermal_Camera.ino) | Reads MLX90640 temperatures and sends frames over serial. |
| [MLX90640_API.h](thermal/MLX90640_Thermal_Camera/MLX90640_API.h) | Declares sensor functions and calibration structures. |
| [MLX90640_API.cpp](thermal/MLX90640_Thermal_Camera/MLX90640_API.cpp) | Reads sensor frames and calculates calibrated temperatures. |
| [MLX90640_I2C_Driver.h](thermal/MLX90640_Thermal_Camera/MLX90640_I2C_Driver.h) | Declares the thermal sensor's I2C interface. |
| [MLX90640_I2C_Driver.cpp](thermal/MLX90640_Thermal_Camera/MLX90640_I2C_Driver.cpp) | Implements I2C register reads and writes. |
| [thermalCamera.hpp](thermal/MLX90640_Thermal_Camera/thermalCamera.hpp) | Declares the thermal adapter used by the integrated counter. |
| [thermalCamera.cpp](thermal/MLX90640_Thermal_Camera/thermalCamera.cpp) | Initializes thermal sensing and combines subpages into complete frames. |
| [Thermal_Config.h](thermal/MLX90640_Thermal_Camera/Thermal_Config.h) | Selects binary or diagnostic serial output. |

## Detection and Viewer

| File | Role |
| --- | --- |
| [blobDetector.hpp](thermal/blobDetector.hpp) | Learns a temperature background and detects connected warm regions. |
| [thermaltest.cpp](thermal/thermaltest.cpp) | Displays serial temperature frames and blob overlays with SDL. |
| [packetReader.hpp](thermal/packetReader.hpp) | Reconstructs temperature packets from serial bytes. |
| [floatPacket.hpp](thermal/floatPacket.hpp) | Defines the temperature packet and payload size. |

## Build

| File | Role |
| --- | --- |
| [Makefile](Makefile) | Builds the viewer and ESP32 firmware and removes generated files. |
| [thermal/Makefile](thermal/Makefile) | Builds the SDL thermal viewer. |

## Makefile Usage

Run these commands from the repository root:

```sh
make                            # Build the SDL viewer
make firmware                   # Build all five firmware variants
make firmware-lidar             # Build dual LiDAR firmware
make firmware-single-lidar      # Build single LiDAR firmware
make firmware-integrated        # Build the integrated counter
make firmware-thermal           # Build thermal binary and diagnostic modes
make stage-integrated           # Collect integrated sketch files without compiling
make test-lidar                 # Run synthetic direction-counter checks without hardware
make clean                      # Remove the build directory
```

The viewer is saved as `build/thermaltest`; firmware output is saved under `build/firmware/`. Firmware targets compile without uploading to the board.

The viewer requires a C++17 compiler and SDL2 with `sdl2-config`. Firmware builds require Arduino CLI, the ESP32 board package, and the SparkFun VL53L5CX library.

The default board is `esp32:esp32:esp32`. Override `FQBN`, `ARDUINO_CLI`, `ARDUINO_DATA`, or `LIDAR_LIBRARY` when needed:

```sh
make firmware-lidar FQBN=esp32:esp32:esp32
make firmware-lidar ARDUINO_CLI="/path/to/arduino-cli"
```
