CXX = clang++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Wpedantic
SDL2_CONFIG ?= sdl2-config
BUILD_DIR = build

.PHONY: all test test-sanitize clean
all: thermaltest

thermaltest: thermaltest.cpp packetReader.hpp floatPacket.hpp blobDetector.hpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $$($(SDL2_CONFIG) --cflags) thermaltest.cpp -o $@ $(LDFLAGS) $$($(SDL2_CONFIG) --libs)

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_DIR)/blobDetector_test: tests/blobDetector_test.cpp blobDetector.hpp packetReader.hpp floatPacket.hpp | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tests/blobDetector_test.cpp -o $@ $(LDFLAGS)

test: $(BUILD_DIR)/blobDetector_test
	./$(BUILD_DIR)/blobDetector_test

test-sanitize: | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) -std=c++17 -O1 -g -Wall -Wextra -Wpedantic -fsanitize=address,undefined -fno-omit-frame-pointer tests/blobDetector_test.cpp -o $(BUILD_DIR)/blobDetector_test_sanitize $(LDFLAGS)
	./$(BUILD_DIR)/blobDetector_test_sanitize

clean:
	rm -f thermaltest $(BUILD_DIR)/blobDetector_test $(BUILD_DIR)/blobDetector_test_sanitize
