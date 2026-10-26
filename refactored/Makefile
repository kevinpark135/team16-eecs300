.PHONY: all test test-sanitize smoke firmware

all:
	$(MAKE) -C thermal VIEWER=../build/thermaltest

test:
	$(MAKE) -C thermal test BUILD_DIR=../build/thermal-tests
	$(MAKE) -C lidar test
	$(MAKE) -C integrated test

test-sanitize:
	$(MAKE) -C thermal test-sanitize BUILD_DIR=../build/thermal-tests
	$(MAKE) -C lidar test-sanitize
	$(MAKE) -C integrated test-sanitize

smoke: all
	python3 thermal/tests/viewer_smoke.py

firmware:
	python3 build_firmware.py
