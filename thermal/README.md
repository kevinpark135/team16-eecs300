# Thermal viewer의 현재 프레임 blob counter

MLX90640의 **원래 섭씨 온도**와 고정된 픽셀별 배경을 비교해서, 현재 32×24 프레임의 따뜻한 연결 영역을 세는 viewer야. 사람 분류기, 방향 추정기, 누적 통과 인원 계산기는 아니야. 매 프레임 min/max로 정규화하는 기존 heatmap 색은 표시용이고 검출 입력으로 쓰지 않아.

## macOS 빌드·실행

레포 루트에서 시작하면 돼. 기존 Homebrew SDL2 또는 SDL2 API를 제공하는 `sdl2-compat` 환경의 `sdl2-config`를 사용해. 추가 폰트·OpenCV 라이브러리는 필요 없어.

```sh
cd thermal
sdl2-config --version --cflags --libs
make
ls /dev/cu.*
./thermaltest /dev/cu.usbserial-실제장치명
```

장치명은 `ls`에 나온 실제 ESP32 serial 장치로 바꿔. Arduino Serial Monitor와 다른 viewer는 먼저 닫아. 포트 점유·열기 실패·연결 끊김은 기존 오류 메시지와 종료 동작을 유지해.

Make 없이 직접 빌드하려면 아래 명령을 써.

```sh
clang++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  $(sdl2-config --cflags) thermaltest.cpp -o thermaltest \
  $(sdl2-config --libs)
```

`sdl2-config`가 PATH에 없다면 기존 설치 경로를 지정할 수 있어.

```sh
make SDL2_CONFIG=/opt/homebrew/bin/sdl2-config
```

ESP32는 기존 binary 모드(`Thermal_Config.h`의 `THERMAL_DIAGNOSTIC_MODE=0`)로 실행해. firmware, 115200 baud, 센서 refresh rate, I2C 설정, `AA BB` + 3072-byte/768-float 패킷 형식은 이번 변경에서 유지했어. 검출 로그는 컴퓨터 viewer가 stderr로 출력해.

```sh
./thermaltest /dev/cu.usbserial-실제장치명 2>thermal-session.log
```

## 보정과 화면

1. viewer를 시작하기 **전에 관측 구역에서 사람을 빼고**, 처음 10개 완성 프레임 동안 빈 장면을 유지해. 벽·고정 물체의 온도를 픽셀별로 수집해.
2. 유효한 샘플의 median으로 배경을 만들고 고정해. NaN/Inf는 샘플에서 제외하고, `ceil(calibrationFrames / 2)`개보다 유효 샘플이 적은 픽셀은 검출에서 제외해. 기본값이면 5개 이상 필요해. 짝수 개 샘플의 median은 중앙 두 값의 평균이야.
3. 보정에 쓴 마지막 프레임도 `CALIBRATING 10/10`으로 표시해. 그 **다음 새 프레임부터** 검출해. 전부 무효인 패킷도 보정 프레임 수에는 들어가므로, 배경이 무효하면 빈 장면과 센서 상태를 확인하고 R로 다시 보정해.
4. 검출한 blob마다 흰 bounding box와 빨간 centroid 십자를 heatmap 위에 그려. 기존 셀 크기는 10×10 화면 픽셀이야. 센서 centroid의 화면 위치는 `((centroidX + 0.5) * 10, (centroidY + 0.5) * 10)`으로 셀 중심을 반영해.
5. **viewer 창에 포커스를 두고 R을 누르면** 배경·샘플·이전 blob·수신 시간 메타데이터를 즉시 지우고 재보정해. 화면도 파란 대기 패턴으로 초기화해. 재보정 동안 다시 빈 장면을 유지해. 세션의 프레임 번호는 계속 증가해.

고정 배경은 사람이 가만히 서 있어도 학습하지 않아. 실내 온도·센서 위치·장면이 달라지면 빈 장면에서 R로 다시 보정해야 해. 사람이 있는 채로 보정하면 그 사람의 온도가 배경에 들어가 검출이 어려워져.

## 설정 조정

설정은 [`blobDetector.hpp`](blobDetector.hpp)의 `DetectorConfig` 한곳에 있어. 수정 후 `make`로 다시 빌드하면 돼.

| 설정 | 기본값 | 의미 |
|---|---|---|
| `calibrationFrames` | `10` | 배경 보정에 사용할 완성 프레임 수, 최소 1 |
| `deltaThresholdC` | `3.0f` | `current - background >= threshold`면 foreground |
| `minBlobArea` | `4` | 이 값보다 작은 연결 영역은 제외 |
| `roi` | `{0, 0, 32, 24}` | 센서 좌표의 `{x, y, width, height}` |
| `staleTimeout` | `std::chrono::milliseconds{2000}` | 새 완성 프레임 없이 기다릴 수 있는 시간 |

예를 들어 중앙만 보려면 `Roi roi{4, 3, 24, 18};`로 바꿔. ROI는 32×24 안의 비어 있지 않은 사각형이어야 해. threshold는 0 이상의 유한 값, 최소 면적과 보정 프레임 수는 양수, timeout은 0보다 커야 해. 잘못된 설정은 검출기 생성 시 거부해.

3°C와 4픽셀은 **실험용 시작값**이야. 사람 검출을 보장하지 않아. 빈 장면에서 잡음이 많으면 threshold나 최소 면적을 올려 보고, 사람이 검출되지 않으면 실제 온도 차이·거리·ROI와 함께 낮추는 실험을 해. ROI 경계에 걸친 물체는 ROI 안쪽 면적만 계산해. 절대 온도 37°C 조건은 없고, blur나 morphology도 기본 적용하지 않아.

## 상태와 개수의 의미

| 상태 | 의미 | 창 제목의 개수 |
|---|---|---|
| `WAITING` | 시작 후 완성 패킷을 아직 받지 못함 | `Blobs: --` |
| `CALIBRATING` | 빈 장면의 배경 샘플 수집·완료 프레임 | `Blobs: --` |
| `READY` | 유효한 입력으로 현재 프레임 검출 완료 | `Blobs: N`, 빈 장면이면 `0` |
| `DEGRADED` | 일부 NaN/Inf 또는 ROI 내 배경 부족 픽셀을 제외하고 검출함 | 사용 가능한 픽셀에서의 `Blobs: N` |
| `INVALID` | ROI에 쓸 수 있는 현재 온도/배경 쌍이 하나도 없음; 보정 중이면 ROI의 유효 샘플이 없음 | `Blobs: --` |
| `STALE` | 기본 2초 이상 새 완성 패킷이 없음; 최초 대기·재보정 대기에도 적용 | `Blobs: --` |

정상 새 프레임에서 blob이 없어지면 즉시 `Blobs: 0`이 돼. `--`는 검출 근거를 제공할 수 없다는 뜻이고, 측정된 0개와 달라. `DEGRADED`의 0개는 부분 입력에서 얻은 결과라서 `READY`의 0개와 상태를 함께 읽어야 해. 보정 중 일부 픽셀이 무효이면 `CALIBRATING ... | DEGRADED`처럼 보정 상태와 데이터 품질을 함께 표시해. ROI 밖에 NaN/Inf가 있어도 프레임 품질은 `DEGRADED`로 남겨.

STALE에서는 이전 blob 표시를 지우고 heatmap만 마지막 프레임으로 남겨. 새 패킷이 오면 해당 프레임 상태로 복구해. 일부 serial 바이트만 들어오는 것은 새 프레임이 아니므로 timeout을 연장하지 않아. 보정 도중 통신이 멈추면 수집한 샘플은 유지하고 재개 시 이어서 보정해.

## 결과 API와 로그

`blobDetector.hpp`는 SDL·serial에 의존하지 않는 C++17 모듈이야. 8방향 BFS로 foreground를 분리하고 면적 필터를 통과한 blob만 반환해. 배열은 `index = row * 32 + column`, 좌표는 `x = column`, `y = row`야. 각 centroid는 해당 blob 픽셀 좌표의 **산술평균**이야.

`BlobDetector::processFrame()`은 새 완성 프레임마다 한 번 호출해. viewer의 `onReceived()`가 `packet.data`를 직접 전달해. `latestResult()`는 마지막 처리 프레임이고, **현재 시점에 근거가 유효한지 확인할 소비자는 `snapshot(now)`를 사용해야 해**. snapshot은 검출을 다시 실행하지 않으며 timeout을 적용해 오래된 blob을 숨겨.

`FrameResult`에 다음 정보를 담았어.

- `frameNumber`: 컴퓨터가 처리한 완성 패킷 번호. 센서가 생성한 번호가 아니야.
- `receivedAt`: `std::chrono::steady_clock` 기반 **컴퓨터의 완성 패킷 수신 시간**. 센서의 실제 촬영 타임스탬프가 아니고, Unix 시각도 아니야. 초기 대기·R 직후에는 값이 없어.
- `status`, `backgroundReady`, `calibrationFramesCollected`, `dataDegraded`: 검출 가능 여부·보정 진행·입력 품질.
- `validPixelCount`, `roiPixelCount`: 보정 중에는 ROI의 유효 현재 샘플 수, 검출 중에는 ROI의 유효 현재/배경 쌍 수. STALE에서 이 메타데이터는 마지막 프레임을 설명해.
- `finitePixelCount`, `invalidPixelCount`: 전체 768픽셀의 유한/NaN·Inf 현재 온도 개수.
- `backgroundUnavailablePixelCount`: 검출 중 ROI의 배경 부족 픽셀 수. `excludedPixelCount`는 ROI에서 현재 무효 또는 배경 부족으로 제외한 픽셀의 합집합 크기라서 중복으로 세지 않아. ROI 밖 무효 픽셀은 `invalidPixelCount`에만 들어가.
- `blobs`: 각 blob의 `label`, `areaPixels`, `centroidX/Y`, `boundingBox`, `meanTemperatureC`, `maxTemperatureC`. bounding box의 min/max 좌표는 양쪽 모두 포함해.

`blobCount()`는 `READY`/`DEGRADED`에서만 개수를 반환하고, 다른 상태에서는 `std::nullopt`를 반환해. label은 프레임마다 1부터 붙이는 지역 번호야. 지속적인 사람 ID가 아니고 추적용으로 쓰면 안 돼. 평균·최고 온도도 색이나 온도 차이가 아니라 blob의 원래 섭씨 값이야.

터미널에는 매 프레임 요약과 blob 상세를 출력해. 예를 들어 배경이 20°C인 합성 프레임의 출력은 아래 형태야. 수신 시간 숫자는 실행마다 달라져.

```text
[thermal] frame=11 received_monotonic_ms=... status=READY blobCount=1 validPixels=768/768 invalidPixels=0 excludedRoiPixels=0 backgroundUnavailable=0
[blob] frame=11 label=1 centroid=(2.25,3.75) areaPixels=4 bbox=(2,3)-(3,5) mean=25.00C max=28.00C
```

기존 `[serial]` 수신 통계와 `[temperature]` min/max 진단도 유지해. 대기·보정·무효 프레임의 로그는 `blobCount=unavailable`이고, STALE 전환은 `[viewer] detection=STALE`로 남겨.

## 하드웨어 없는 검증

```sh
make test
make test-sanitize
```

검출기 테스트는 SDL이나 serial 장치 없이 돌아가. 두 번째 명령은 AddressSanitizer와 UndefinedBehaviorSanitizer를 적용해. 합성 테스트가 확인하는 항목은 다음과 같아.

- 배경만 있을 때 0개, 따뜻한 영역 1개, 떨어진 두 영역 2개, 최소 면적 미만 잡음 제외.
- 정확한 centroid·포함형 bounding box·면적·평균/최고 온도, 3°C 경계값 포함, 37°C 미만 검출.
- NaN/Inf 제외, 보정 중 데이터 품질, 유효 샘플 median, 부족한 배경 샘플 제외, 전체 무효 상태와 복구.
- 정지 blob을 200프레임 유지해도 배경으로 흡수되지 않음.
- 대각선 8방향 연결, 따뜻한 다리로 연결된 두 영역이 하나가 되는 한계, 이미지 가장자리와 768픽셀 BFS.
- ROI·설정 변경, 새 빈 프레임에서 즉시 0개, 최초 대기·검출 후 timeout과 새 프레임 복구.
- 재보정 시 이전 결과·배경·수신 시각 초기화, 새 배경 사용.
- 기존 packetReader에 분할 패킷을 넣어 마지막 바이트가 도착할 때만 프레임당 한 번 실행.

## 실제 센서로 확인할 순서

각 실험의 장면·거리·온도·설정과 로그를 함께 기록해. 아래 기대값은 측정 목표이고 사람 수 보장은 아니야.

| 장면 | 확인할 내용 |
|---|---|
| 빈 장면 | 처음 10프레임 보정 후 `READY / Blobs: 0`인지, 빈 장면에서 잡음 blob이 생기는지 |
| 한 사람 | ROI 안에서 실제 배경 대비 온도 차이와 최소 면적이 충분한지, bbox/십자가 몸의 따뜻한 영역에 놓이는지, 가만히 서 있어도 유지되는지 |
| 떨어진 두 사람 | 센서 해상도에서 두 따뜻한 영역 사이에 foreground가 아닌 셀이 있는지, 분리되면 2개가 나오는지 |
| 붙은 두 사람 | 영역이 직접 또는 대각선으로 연결되면 1개로 합쳐지는지, 이 한계를 기록하기 |
| 따뜻한 물체 | 컵·히터 같은 물체도 threshold·면적을 만족하면 blob으로 세는지 |
| 사람이 나간 직후 | 다음 정상 프레임에서 count가 바로 0으로 돌아오는지 |
| R 재보정 | 이전 bbox/십자가 즉시 사라지고 빈 장면에서 보정을 다시 하는지 |
| 수신 중단 | 연결은 유지한 채 ESP32 송신을 잠시 멈추면 2초 뒤 `STALE / Blobs: --`와 overlay 제거가 되는지; USB를 뽑으면 기존 연결 끊김 오류로 종료함 |

한 사람이 여러 따뜻한 영역으로 나뉘면 여러 blob이 될 수 있고, 두 사람이 붙으면 한 blob이 될 수 있어. 옷·거리·배경 온도·32×24 해상도 때문에 사람이 있어도 검출되지 않을 수 있어. **blob 수와 실제 사람 수가 항상 같지는 않아.**

## 이후 LiDAR 통합에 필요한 것

현재 레포에는 LiDAR 이벤트와 컴퓨터의 thermal 결과를 연결하는 통신 경로가 없어. `lidar/direction_sensing.ino`는 감지한 셀 전체의 거리 가중 centroid 하나와 방향 누적값을 사용하고, `lidar/2LiDARs.ino`는 두 거리 격자를 serial 텍스트로 출력해. 이번 thermal 결과는 프레임별 분리된 blob의 산술 centroid이고, LiDAR 코드나 firmware는 수정하지 않았어.

실제 통합에는 공통 관측 구역과 좌표 대응을 확인하고, LiDAR 이벤트에 어떤 thermal 프레임을 대응시킬지 이벤트 시간창을 정하고, 센서별 시간 기준·전송 지연을 정렬하는 절차가 필요해. MCU의 `millis()`와 컴퓨터의 `steady_clock`은 바로 비교할 수 없어. thermal 패킷에는 센서 촬영 시각이 없으므로 컴퓨터 수신 시각을 촬영 시각으로 간주하면 안 돼. 이 프로토콜에는 checksum도 없어서 유한 값으로 손상된 payload까지 검출기가 판별할 수 있는 것은 아니야.

통합 소비자는 상태·freshness·제외 픽셀 수와 blob의 위치를 함께 평가해야 해. LiDAR 이벤트가 두 개이고 thermal blob이 하나라는 관측만으로 실제 사람 수를 1로 강제 수정하지 않아. 이번 결과에는 `humanCount`, 방향 추정, 누적 통과 인원, 사람 추적 ID가 없어.
