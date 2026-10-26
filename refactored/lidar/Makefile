CXX = clang++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Wpedantic
BUILD_DIR = ../build/lidar-tests

.PHONY: test test-sanitize

test: $(BUILD_DIR)/counters_test
	./$(BUILD_DIR)/counters_test

$(BUILD_DIR)/counters_test: tests/counters_test.cpp directionCounter.hpp laneCounter.hpp
	mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDFLAGS)

test-sanitize:
	mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) -std=c++17 -O1 -g -Wall -Wextra -Wpedantic -fsanitize=address,undefined -fno-omit-frame-pointer tests/counters_test.cpp -o $(BUILD_DIR)/counters_test_sanitize $(LDFLAGS)
	./$(BUILD_DIR)/counters_test_sanitize
