#pragma once

#include <Arduino.h>

// Size of the ring buffer — adjust as needed
#define RTCM_BUFFER_SIZE 4096

// Initializes the RTCM ring buffer
void initRtcmBuffer();

// Adds raw bytes to the buffer (returns number written)
size_t writeRtcmData(const uint8_t* data, size_t len);

// Reads up to maxLen bytes from buffer (returns actual read count)
size_t readRtcmData(uint8_t* outBuffer, size_t maxLen);

// Returns number of bytes available to read
size_t getRtcmBytesAvailable();

size_t getDroppedRtcmBytes();
