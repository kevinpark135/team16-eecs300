#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include "floatPacket.hpp"

// Keep parser state between reads because serial chunks may split a packet.
class packetReader {
public:
    // MODIFIES: Parser state and counters.
    // EFFECTS: Returns true only when this byte completes an AA BB + float frame.
    bool processByte(std::uint8_t byte) {
        ++totalBytes;
        switch (state) {
        case WAIT_HEADER1:
            // Ignore incoming bytes until the first header byte appears.
            if (byte == HEADER1) {
                ++header1Candidates;
                state = WAIT_HEADER2;
            }
            break;
        case WAIT_HEADER2:
            if (byte == HEADER2) {
                ++headersFound;
                bytesRead = 0;
                state = READ_PAYLOAD;
            } else if (byte == HEADER1) {
                ++header1Candidates; // AA AA BB still contains a valid header.
            } else {
                state = WAIT_HEADER1;
            }
            break;
        case READ_PAYLOAD:
            // Header-like bytes inside the fixed-size payload are ordinary data.
            payload[bytesRead] = byte;
            ++bytesRead;
            if (bytesRead == THERMAL_PAYLOAD_SIZE) {
                // Copy bytes into floats without reading an inactive union member.
                std::memcpy(currentPacket.data, payload, sizeof(currentPacket.data));
                ++packetsCompleted;
                state = WAIT_HEADER1;
                return true;
            }
            break;
        }
        return false;
    }

    // REQUIRES: A packet has completed; valid until the next completed packet.
    const floatPacket &packet() const { return currentPacket; }
    std::uint64_t getTotalBytes() const { return totalBytes; }
    std::uint64_t getHeader1Candidates() const { return header1Candidates; }
    std::uint64_t getHeadersFound() const { return headersFound; }
    std::uint64_t getPacketsCompleted() const { return packetsCompleted; }
    std::size_t getPayloadBytesRead() const {
        if (state == READ_PAYLOAD) { return bytesRead; }
        return 0;
    }

    const char *getStateName() const {
        switch (state) {
        case WAIT_HEADER1: return "waiting-header-1";
        case WAIT_HEADER2: return "waiting-header-2";
        case READ_PAYLOAD: return "reading-payload";
        }
        return "unknown";
    }

private:
    enum State { WAIT_HEADER1, WAIT_HEADER2, READ_PAYLOAD };
    static const std::uint8_t HEADER1 = 0xAA;
    static const std::uint8_t HEADER2 = 0xBB;
    State state = WAIT_HEADER1;
    std::size_t bytesRead = 0;
    // Publish a new float packet only after this byte buffer is complete.
    std::uint8_t payload[THERMAL_PAYLOAD_SIZE] = {};
    floatPacket currentPacket;
    std::uint64_t totalBytes = 0;
    std::uint64_t header1Candidates = 0;
    std::uint64_t headersFound = 0;
    std::uint64_t packetsCompleted = 0;
};
