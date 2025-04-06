// ubx_parser.h
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// UBX NAV-PVT message length
#define UBX_NAV_PVT_LEN 92

// UBX sync chars
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

// NAV-PVT class and ID
#define UBX_CLASS_NAV 0x01
#define UBX_ID_NAV_PVT 0x07

// Parser return code
typedef enum {
    UBX_RESULT_NONE,
    UBX_RESULT_PVT_READY,
    UBX_RESULT_ERROR
} ubx_result_t;

// Minimal NAV-PVT struct (can expand later)
typedef struct {
    uint32_t iTOW;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint8_t fixType;
    uint8_t flags;
    uint8_t numSV;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
} UBX_NAV_PVT_data_t;

// Callback type
typedef void (*ubx_pvt_callback_t)(const UBX_NAV_PVT_data_t* pvt);

// Set callback
void ubx_set_pvt_callback(ubx_pvt_callback_t cb);

// Feed a byte into the parser
ubx_result_t ubx_parse_byte(uint8_t byte);

#ifdef __cplusplus
}
#endif
