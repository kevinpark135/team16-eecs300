# EECS 280 수준으로 정리한 버전

모든 변경과 새 파일은 `refactored/` 안에 있어. 280 내용을 전부 배웠다는 전제로,
명시적인 반복문, 작은 함수, 배열, 구조체·클래스와 기본 컨테이너를 사용했어.
설명에 필요한 위치에는 짧은 영어 주석을 넣었어.

## LiDAR 읽는 순서

- `lidar/2LiDARs.ino`: 두 센서에서 프레임을 읽고 두 개의 8×8 표를 출력해.
- `lidar/direction_sensing.ino`: 한 센서의 프레임을 받아 방향 계산기에 전달해.
- `lidar/directionCounter.hpp`: 배경 학습 → 물체 중심 → 최근 위치 기록 → 속도 추정 → 방향 확인 → 한 번 카운트.
- `lidar/2LiDARs_direction/2LiDARs_direction.ino`: 두 센서의 왼쪽·오른쪽 열을 각각 처리해.
- `lidar/laneCounter.hpp`: 각 통로에서 어느 센서가 먼저 감지했는지 `enum`과 `switch`로 기억해.
- `lidar/lidarCommon.hpp`: 핀·주소·센서 설정과 공통 입출력 함수를 모았어.

방향 계산과 통로 상태 전이는 Arduino 라이브러리 없이 테스트할 수 있어.
기존 핀, 주소, 거리 기준, 확인 프레임 수와 카운트 조건은 유지했어.
방향 계산에서는 타이머 값을 **먼저 뺀 뒤** 실수로 바꿔서 긴 실행 시간에도
밀리초 정밀도를 유지하도록 했어. 열 번호가 증가하면 오른쪽으로 취급해.
실제 좌우는 센서 설치 방향에 따라 확인해야 해.

두 센서 방식은 기존처럼 두 번째 센서 감지를 타임아웃보다 먼저 확인해.
따라서 두 번째 감지가 늦게 도착한 프레임에도 통과를 보고할 수 있고,
통로가 완전히 비워질 때까지 기다리는 별도의 상태는 없어.
한 센서 방식 역시 물체가 여러 개면 하나의 중심으로 합쳐서 처리해.

## Thermal 읽는 순서

1. `thermal/MLX90640_Thermal_Camera/MLX90640_Thermal_Camera.ino`:
   초기화·교정 → 두 subpage 읽기 → 완성된 온도 프레임 전송.
2. 같은 폴더의 `MLX90640_I2C_Driver.cpp`:
   짧은 I2C 읽기와 16비트 word/byte 변환을 담당해.
3. `thermal/packetReader.hpp`:
   바이트 하나씩 처리하고, 패킷이 완성되면 `true`를 반환해.
4. `thermal/blobDetector.hpp`:
   배경 중앙값 → 온도 차이 → 8방향으로 연결된 픽셀 묶음 찾기.
5. `thermal/thermaltest.cpp`:
   `ThermalViewer`가 시리얼·SDL 자원을 소유하고 수신·감지·색상·출력을 연결해.

통신 형식은 기존 **`AA BB` + float 768개(3072바이트)** 그대로야.
ESP32와 현재 호스트의 32비트 float 바이트 표현을 사용해.
진단 텍스트는 `THERMAL_DIAGNOSTIC_MODE=1`일 때만 전송해.
`0x8000` 상태 레지스터는 센서가 값을 바꾸므로 쓰기 후 값의 일치를 강제하지 않아.

이전 union 대신 수신 바이트를 `memcpy`로 float 배열에 복사해.
콜백·캡처 람다·`std::function`과 viewer의 수동 `new/delete`는 제거했어.
감지 결과는 `getBlobCount(count)`가 `true`일 때만 사용할 수 있어.
아직 결과가 없는 경우와 실제 개수가 0인 경우를 구분해.
재교정은 `R`, 기본 배경 교정은 10프레임, 데이터 만료 시간은 2초야.

제조사 `MLX90640_API.cpp`는 이미 함수·배열·포인터·반복문 중심이라
원본 그대로 유지했어. 센서 교정식과 레지스터 마스크를 일반 로직처럼
다시 설계하면 온도 계산이 바뀔 수 있어. 이 파일의 수식은 센서 전용 지식이야.

## 280 밖에서 필요한 부분

- `Wire`, GPIO, 센서 드라이버: 실제 하드웨어와 통신하는 API야. 입출력 함수에 모았어.
- `termios`, `poll`, SDL: 호스트의 시리얼 설정·대기·화면 출력 API야.
  시리얼 설정은 `openSerialPort()` 안에 있어.
- `std::chrono::steady_clock`: 시스템 시각 변경의 영향을 받지 않고 경과 시간을 재.
- `std::array`: 길이가 고정된 배열에 `fill()`, `size()`, `data()`를 제공하는 기본 래퍼야.
- `reinterpret_cast`: 전송할 float 저장 공간을 바이트로 읽을 때만 사용해.
- `= delete`: SDL 자원의 소유권이 복사되지 않도록 viewer 복사를 막아.

## 빌드와 테스트

저장소 루트에서 실행해. 데스크톱에는 C++17 컴파일러와 `sdl2-config`가 필요해.

```sh
make -C refactored all
make -C refactored test
make -C refactored test-sanitize
make -C refactored smoke
```

빌드 결과는 `refactored/build/`에 생겨. `smoke`는 실제 viewer를 가상 시리얼
포트와 화면 없는 SDL 드라이버로 실행해. 센서를 연결하거나 펌웨어를 올리지 않아.
실제 viewer 실행은 현재 장치 경로를 확인한 다음 아래 형식으로 해.
처음 10프레임 동안 장면을 비워 둬.

```sh
./refactored/build/thermaltest <serial-device>
```

복사본에 원래 들어 있던 실행 파일 대신 위 경로의 새 빌드 결과를 사용해.

ESP32 빌드는 설치된 Arduino IDE/CLI, ESP32 core, SparkFun VL53L5CX 라이브러리를 사용해.
현재 스크립트의 설치 경로는 이 Mac 기준이야. 타깃은 일반 ESP32로 지정했어.
이 빌드만으로 실제 보드 모델이나 배선이 확인되는 건 아니야.

```sh
python3 refactored/build_firmware.py
# 필요하면 실제 보드의 FQBN과 CLI 경로를 지정해.
python3 refactored/build_firmware.py --fqbn esp32:esp32:esp32 --cli /path/to/arduino-cli
```

LiDAR의 세 `.ino`는 서로 다른 프로그램이야. Arduino가 한 폴더의 `.ino`를
합치지 않도록 스크립트가 각각 독립된 sketch 폴더에 복사하고 공통 헤더도 함께 넣어.
IDE로 올릴 때도 `refactored/build/sketches/`의 해당 sketch를 사용해.
이 스크립트는 컴파일만 하고 업로드하지 않아.

검증한 범위는 thermal 테스트 16개, LiDAR 테스트 6개 묶음,
AddressSanitizer/UndefinedBehaviorSanitizer, 가상 시리얼 viewer 실행,
LiDAR 3개와 thermal 2가지 모드의 ESP32 컴파일이야.
제조사 계산 코드에는 기존 타입 범위 경고가 남아 있어.
실제 센서 측정·사람 통과·배선·보드 업로드는 이번 검증에 포함하지 않았어.
