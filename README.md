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
| [2LiDARs_direction.ino](lidar/2LiDARs_direction/2LiDARs_direction.ino) | Reads two LiDARs into one 16x8 field without a display divider. Prints one shared IN/OUT count. |
| [combinedField.hpp](lidar/2LiDARs_direction/combinedField.hpp) | Rotates and pairs new sensor frames, then feeds the 128-cell multi-object detector. |
| [multiObjectCounter.hpp](lidar/2LiDARs_direction/multiObjectCounter.hpp) | Builds a foreground mask, finds connected regions, and tracks up to two centroids with independent direction histories and counted states. |
| [directionCounter.hpp](lidar/2LiDARs_direction/directionCounter.hpp) | Retains the original one-centroid counter for the existing integrated sketch and supplies shared detection thresholds. |
| [directionSettings.hpp](lidar/2LiDARs_direction/directionSettings.hpp) | Sets per-sensor mounting directions for the existing integrated sketch. |
| [lidarCommon.hpp](lidar/2LiDARs_direction/lidarCommon.hpp) | Defines hardware settings, initializes both LiDARs, and validates readings. |
| [directionCounter_test.cpp](lidar/2LiDARs_direction/tests/directionCounter_test.cpp) | Checks fast passages, mounting direction, noise rejection, and duplicate-count prevention using synthetic frames. |
| [multiObjectCounter_test.cpp](lidar/2LiDARs_direction/tests/multiObjectCounter_test.cpp) | Checks paired frames, region extraction, seam handoffs, two-object association, simultaneous IN/OUT events, dropouts, reset, and timer wrap using synthetic frames. |

The dual-LiDAR sketch uses the displayed field's vertical axis: downward movement is IN,
upward movement is OUT, preserving the previous mounting convention. These labels must
be checked against the physical entry/exit direction. Counting still uses the existing
confirmed-motion thresholds, not a crossing line. The scene must be empty for 15 paired startup
frames. Each detection update needs a new frame from both sensors, with host receive
times no more than 100 ms apart; this does not synchronize the sensors' exposures.
`R` clears counts and restarts calibration. The grid displays the latest readings
independently, with `X` for invalid cells and `--` for frames older than 500 ms.

The detector groups foreground cells using eight-neighbor connectivity across the entire
field, including the sensor seam. Each region needs at least three cells. Its centroid
uses the existing inverse-distance weights; its bounds and mean distance are also recorded.
No head shape or person-height model is used. The two-object tracker matches detections
using predicted 2D positions, with a five-cell association radius around either the prediction
or the last observed position. It tests all assignments to avoid assigning one detection to
two tracks. Each track owns its direction history and can count once while it remains active.
Five unmatched paired frames, or more than 500 ms without an observation, retire that track
without clearing the other track or the running counts. A brief loss clears pending direction
evidence but retains the counted state. Lost tracks reserve their slots until retirement.

The serial report defaults to one spaced summary line:
`RUNNING COUNT | IN: #              OUT: #              CENTROID: #`.
Set `SHOW_FOV = true` near the top of `2LiDARs_direction.ino` and rebuild/upload
to show the 16x8 distance grid above the summary. The default `false` hides only
the grid; sensor acquisition, tracking, and counting continue unchanged.
`CENTROID` is the number of detected regions in the latest paired frame (up to two),
not the number of retained tracks. It shows `--` when that frame is older than 500 ms.
Detailed region and track diagnostics are hidden. If more than two valid regions appear,
the detector retains the largest two candidates and pauses counting for that frame.

This version targets spatially separated objects. Touching regions are still one detection;
merge/split recovery and thermal fusion are not implemented. Region count is not guaranteed
to equal people count. Synthetic tests do not establish physical two-person or wheelchair accuracy.

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

## Important CLIs

Run these commands from the repository root:

### Thermal Viewer: Compile and Run

```sh
make viewer                     # Compile thermaltest (same as make)
ls /dev/cu.*                    # Find the serial device on macOS
./build/thermaltest /dev/cu.usbserial-0001
```

Replace `/dev/cu.usbserial-0001` with the device listed for your board.
To compile and launch in one command:

```sh
make viewer && ./build/thermaltest /dev/cu.usbserial-0001
```

The viewer needs the thermal binary firmware (`make firmware-thermal-binary`),
uploaded to the board. Close Arduino Serial Monitor before opening the viewer
so only one program uses the serial port.

### Main Makefile Commands

```sh
make                            # Build the SDL viewer
make viewer                     # Explicitly build the SDL viewer
make firmware                   # Build all five firmware variants
make firmware-lidar             # Build dual LiDAR firmware
make firmware-single-lidar      # Build single LiDAR firmware
make firmware-integrated        # Build the integrated counter
make firmware-thermal           # Build thermal binary and diagnostic modes
make firmware-thermal-binary    # Build thermal firmware for the SDL viewer
make firmware-thermal-diagnostic # Build human-readable thermal diagnostics
make stage-integrated           # Collect integrated sketch files without compiling
make test-lidar                 # Run single- and multi-object synthetic LiDAR checks
make clean                      # Remove the build directory
```

The viewer is saved as `build/thermaltest`; firmware output is saved under `build/firmware/`. Firmware targets compile without uploading to the board.

The viewer requires a C++17 compiler and SDL2 with `sdl2-config`. Firmware builds require Arduino CLI, the ESP32 board package, and the SparkFun VL53L5CX library.

The default board is `esp32:esp32:esp32`. Override `FQBN`, `ARDUINO_CLI`, `ARDUINO_DATA`, or `LIDAR_LIBRARY` when needed:

```sh
make firmware-lidar FQBN=esp32:esp32:esp32
make firmware-lidar ARDUINO_CLI="/path/to/arduino-cli"
```
