# Integrated ESP32 People Counter

One ESP32 reads LiDAR L, LiDAR R, and the MLX90640 thermal camera. Both LiDARs run the existing background/centroid/speed direction algorithm independently. Thermal blob detection runs locally on the ESP32 using the same detector as the desktop viewer.

## Decision Rules

| Stable thermal count | Direction events within one matching window | Result |
| --- | --- | --- |
| 1 | Both ENTER | One entry |
| 1 | Both EXIT | One exit |
| 1 | Disagreement or a missing direction | No count; rejected at the end of the window |
| 2 | Both ENTER | Two entries |
| 2 | ENTER and EXIT | One entry and one exit; occupancy change is zero |
| 2 | Both EXIT | Two exits |
| 2 | Only one confirmed direction | Count that confirmed side only; do not invent the other direction |
| 0, above 2, invalid, or stale | Any | No automatic count |

Each LiDAR emits an event only when its direction first becomes confirmed. The passage counter collects these events, then decides when the scene stays clear or the matching window expires. Decisions may therefore appear later than individual direction candidates.

A later stable count of 2 upgrades a pending one-person passage. A later count of 1 does not downgrade it. A single-frame count change does not select a new mode. Stable counts above 2 or conflicting directions from the same side reject the passage. Zero frames can end a passage without erasing its recent positive thermal evidence.

After every decision, fresh thermal zero frames and fresh empty readings from both LiDARs must persist for the clear interval before the next passage is accepted. Continuous traffic without a clear interval is outside this prototype's supported behavior. Each passage accepts at most one direction per LiDAR.

## Settings to Check on the Mounted Unit

- `../lidar/directionSettings.hpp`: `axis` selects `COLUMNS` or `ROWS`; `reverse` flips that sensor's positive direction. Positive configured movement maps to ENTER, negative to EXIT. Both defaults use columns without reversal and are unverified placeholders.
- `../lidar/directionCounter.hpp`: background learning, distance difference, minimum cells, speed, travel, confirmation, and loss thresholds retain the existing direction algorithm's defaults. Its direction is a motion estimate, not an independent proof of crossing the doorway.
- `systemConfig.hpp`: matching window, thermal/LiDAR freshness, clear interval, thermal stability, and starting occupancy. Initial defaults are 2000/1500/500/300 ms and two stable thermal frames. These need physical tuning.
- The same I2C pins and addresses as the existing code are used: SDA 21, SCL 22, LiDAR LPn 26/19, LiDAR addresses 0x30/0x32, thermal address 0x33, shared bus 400 kHz.

Keep all views empty until both LiDAR backgrounds and the thermal background are calibrated. Wait for `Lcal=1 Rcal=1 thermal=0` in the status output before walking through. The LiDAR counters collect 15 frames; the thermal detector collects 10 complete temperature frames. The configured sensor frequencies are not a measured integrated processing rate.

## Firmware and Output

From the repository root:

```sh
python3 refactored/build_firmware.py --target integrated
make -C refactored test
make -C refactored test-sanitize
```

The build script stages the complete sketch at `refactored/build/sketches/integrated/`. Open its `integrated.ino` in Arduino IDE to upload to the actual board. The script only compiles; its default build target is generic `esp32:esp32:esp32`. Change `--fqbn` if the actual board requires another target.

The integrated program prints readable candidate events, counted/rejected passage decisions, and periodic status at 115200 baud. `thermal=-1` means unavailable or stale. `thermalError` reports the last sensor read error (zero after a successful frame), and `freeHeap` reports available heap memory for hardware checks. State numbers are `0=IDLE`, `1=COLLECTING`, `2=WAIT_CLEAR`. `occupancy` is starting occupancy + entries - exits; it can be negative if the starting occupancy is wrong. Values are not silently clamped.

This integrated output is separate from the standalone thermal sketch's unchanged `AA BB` + 3072-byte binary protocol. The SDL viewer continues to use the standalone thermal sketch. Do not feed the integrated text output to that viewer.

## Scope of the Prototype

The intended assumption is that one thermal blob represents one person seen by both LiDARs, while two thermal blobs represent one person per LiDAR. Mounting and real scenes must validate this assumption. The code does not perform person identification, resolve merged thermal blobs, or separate multiple people inside one LiDAR view.

Thermal reads return immediately when no new subpage is ready; I2C transfers and bounded retries still take time. Both halves must arrive within the adapter timeout. Failed or incomplete pairs are discarded, and degraded thermal frames are not used for occupancy decisions. Sensor read gaps expire fusion evidence and reset stale LiDAR movement history.

Desktop tests use synthetic measurements and simulated driver calls. ESP32 compilation verifies toolchain compatibility, not physical accuracy, heap availability under all conditions, actual frame rates, mounting direction, or real passage counts. Those checks require the assembled hardware.
