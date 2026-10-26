CXX = clang++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Wpedantic
SDL2_CONFIG ?= sdl2-config
VIEWER = ../build/thermaltest

.PHONY: all clean
all: $(VIEWER)

$(VIEWER): thermaltest.cpp packetReader.hpp floatPacket.hpp blobDetector.hpp
	mkdir -p $(dir $(VIEWER))
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $$($(SDL2_CONFIG) --cflags) thermaltest.cpp -o $@ $(LDFLAGS) $$($(SDL2_CONFIG) --libs)

clean:
	rm -f $(VIEWER)
