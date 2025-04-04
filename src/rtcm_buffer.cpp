#include "rtcm_buffer.h"

// Internal ring buffer state
static uint8_t buffer[RTCM_BUFFER_SIZE];
static volatile size_t head = 0;
static volatile size_t tail = 0;
static size_t droppedRtcmBytes = 0;

// Simple helper macro
#define BUFFER_MASK (RTCM_BUFFER_SIZE - 1)

void initRtcmBuffer() {
    head = tail = 0;
}

size_t getRtcmBytesAvailable() {
    return (head >= tail) ? (head - tail) : (RTCM_BUFFER_SIZE - (tail - head));
}

size_t writeRtcmData(const uint8_t* data, size_t len) {
    size_t written = 0;
    for (size_t i = 0; i < len; i++) {
        size_t next = (head + 1) % RTCM_BUFFER_SIZE;
        if (next == tail){
            droppedRtcmBytes += (len - written); // count how much we couldn’t write
            break; // Buffer full
        } 
        
        buffer[head] = data[i];
        head = next;
        written++;
    }
    return written;
}

size_t readRtcmData(uint8_t* outBuffer, size_t maxLen) {
    size_t read = 0;
    while (tail != head && read < maxLen) {
        outBuffer[read++] = buffer[tail];
        tail = (tail + 1) % RTCM_BUFFER_SIZE;
    }
    return read;
}

size_t getDroppedRtcmBytes() {
    return droppedRtcmBytes;
}
