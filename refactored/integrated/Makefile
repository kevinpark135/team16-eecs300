CXX = clang++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Wpedantic
BUILD_DIR = ../build/integrated-tests
CAMERA_DIR = ../thermal/MLX90640_Thermal_Camera
SANITIZERS = -std=c++17 -O1 -g -Wall -Wextra -Wpedantic -fsanitize=address,undefined -fno-omit-frame-pointer

.PHONY: test test-sanitize

test: $(BUILD_DIR)/passageCounter_test $(BUILD_DIR)/frameReader_test $(BUILD_DIR)/thermalCamera_test
	./$(BUILD_DIR)/passageCounter_test
	./$(BUILD_DIR)/frameReader_test
	./$(BUILD_DIR)/thermalCamera_test

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_DIR)/passageCounter_test: tests/passageCounter_test.cpp passageCounter.hpp ../lidar/directionCounter.hpp ../thermal/blobDetector.hpp | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDFLAGS)

$(BUILD_DIR)/frameReader_test: tests/frameReader_test.cpp $(CAMERA_DIR)/MLX90640_API.cpp $(CAMERA_DIR)/MLX90640_API.h tests/fakes/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Itests/fakes tests/frameReader_test.cpp $(CAMERA_DIR)/MLX90640_API.cpp -o $@ $(LDFLAGS)

$(BUILD_DIR)/thermalCamera_test: tests/thermalCamera_test.cpp $(CAMERA_DIR)/thermalCamera.cpp $(CAMERA_DIR)/thermalCamera.hpp tests/fakes/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Itests/fakes tests/thermalCamera_test.cpp $(CAMERA_DIR)/thermalCamera.cpp -o $@ $(LDFLAGS)

test-sanitize: | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(SANITIZERS) tests/passageCounter_test.cpp -o $(BUILD_DIR)/passageCounter_test_sanitize $(LDFLAGS)
	./$(BUILD_DIR)/passageCounter_test_sanitize
	$(CXX) $(CPPFLAGS) $(SANITIZERS) -Itests/fakes tests/frameReader_test.cpp $(CAMERA_DIR)/MLX90640_API.cpp -o $(BUILD_DIR)/frameReader_test_sanitize $(LDFLAGS)
	./$(BUILD_DIR)/frameReader_test_sanitize
	$(CXX) $(CPPFLAGS) $(SANITIZERS) -Itests/fakes tests/thermalCamera_test.cpp $(CAMERA_DIR)/thermalCamera.cpp -o $(BUILD_DIR)/thermalCamera_test_sanitize $(LDFLAGS)
	./$(BUILD_DIR)/thermalCamera_test_sanitize
