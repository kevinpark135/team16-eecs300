ARDUINO_CLI ?= $(shell command -v arduino-cli 2>/dev/null)
ifeq ($(strip $(ARDUINO_CLI)),)
ARDUINO_CLI := /Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli
endif
FQBN ?= esp32:esp32:esp32
ARDUINO_DATA ?= $(HOME)/Library/Arduino15
LIDAR_LIBRARY ?= $(HOME)/Documents/Arduino/libraries/SparkFun_VL53L5CX_Arduino_Library
BUILD_DIR := $(CURDIR)/build
ARDUINO_CONFIG := $(BUILD_DIR)/arduino-cli.yaml
THERMAL_SOURCE := thermal/MLX90640_Thermal_Camera
LIDAR_SOURCE := lidar/2LiDARs_direction
INTEGRATED_SKETCH := $(BUILD_DIR)/sketches/integrated

ARDUINO_COMPILE = TMPDIR="$(BUILD_DIR)/tmp" PYTHONPYCACHEPREFIX="$(BUILD_DIR)/python-cache" \
	"$(ARDUINO_CLI)" compile --config-file "$(ARDUINO_CONFIG)" --fqbn "$(FQBN)" \
	--warnings all --library "$(LIDAR_LIBRARY)"

.PHONY: all viewer firmware firmware-lidar firmware-single-lidar firmware-integrated \
	firmware-thermal firmware-thermal-binary firmware-thermal-diagnostic \
	prepare-firmware stage-integrated clean

all: viewer

viewer:
	$(MAKE) -C thermal

firmware: firmware-lidar firmware-single-lidar firmware-thermal firmware-integrated

# Reuse installed board tools; keep CLI state and caches inside build/.
prepare-firmware:
	@test -x "$(ARDUINO_CLI)" || { echo "Arduino CLI not found; set ARDUINO_CLI to its executable path." >&2; exit 1; }
	@test -d "$(ARDUINO_DATA)/packages" || { echo "Installed Arduino packages not found; set ARDUINO_DATA." >&2; exit 1; }
	@mkdir -p "$(BUILD_DIR)/arduino-data" "$(BUILD_DIR)/arduino-user" "$(BUILD_DIR)/tmp"
	@for index in "$(ARDUINO_DATA)"/*index.json*; do \
		if [ -f "$$index" ]; then cp "$$index" "$(BUILD_DIR)/arduino-data/"; fi; \
	done
	@ln -sfn "$(ARDUINO_DATA)/packages" "$(BUILD_DIR)/arduino-data/packages"
	@printf '%s\n' \
		'directories:' \
		'  data: "$(BUILD_DIR)/arduino-data"' \
		'  downloads: "$(BUILD_DIR)/arduino-downloads"' \
		'  user: "$(BUILD_DIR)/arduino-user"' \
		'  builtin:' \
		'    libraries: "$(ARDUINO_DATA)/libraries"' \
		'build_cache:' \
		'  path: "$(BUILD_DIR)/arduino-cache"' > "$(ARDUINO_CONFIG)"

firmware-lidar: prepare-firmware
	$(ARDUINO_COMPILE) --build-path "$(BUILD_DIR)/firmware/2LiDARs_direction" "$(LIDAR_SOURCE)"

firmware-single-lidar: prepare-firmware
	$(ARDUINO_COMPILE) --build-path "$(BUILD_DIR)/firmware/direction_sensing" "lidar/direction_sensing"

firmware-thermal: firmware-thermal-binary firmware-thermal-diagnostic

firmware-thermal-binary: prepare-firmware
	$(ARDUINO_COMPILE) --build-path "$(BUILD_DIR)/firmware/MLX90640_Thermal_Camera" "$(THERMAL_SOURCE)"

firmware-thermal-diagnostic: prepare-firmware
	$(ARDUINO_COMPILE) --build-path "$(BUILD_DIR)/firmware/thermal-diagnostic" \
		--build-property 'compiler.cpp.extra_flags=-DTHERMAL_DIAGNOSTIC_MODE=1' "$(THERMAL_SOURCE)"

# The integrated sketch needs shared headers and thermal drivers in one folder.
stage-integrated: prepare-firmware
	rm -rf "$(INTEGRATED_SKETCH)"
	mkdir -p "$(INTEGRATED_SKETCH)"
	cp integrated/*.ino integrated/*.hpp "$(LIDAR_SOURCE)"/*.hpp thermal/blobDetector.hpp \
		"$(THERMAL_SOURCE)"/*.h "$(THERMAL_SOURCE)"/*.hpp "$(THERMAL_SOURCE)"/*.cpp "$(INTEGRATED_SKETCH)/"

firmware-integrated: stage-integrated
	$(ARDUINO_COMPILE) --build-path "$(BUILD_DIR)/firmware/integrated" "$(INTEGRATED_SKETCH)"

clean:
	rm -rf "$(BUILD_DIR)"
