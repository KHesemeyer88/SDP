// ubx_parser.h
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- UBX NAV-PVT message structure (subset) ---
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
    uint8_t month, day, hour, min, sec;
    int32_t gSpeed;   // mm/s
    int32_t headMot;  // heading * 1e-5 deg
} UBX_NAV_PVT_data_t;

// --- UBX Constants ---
#define UBX_NAV_PVT_LEN 92
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62
#define UBX_CLASS_NAV 0x01
#define UBX_ID_NAV_PVT 0x07

// --- Parser return results ---
typedef enum {
    UBX_RESULT_NONE,
    UBX_RESULT_PVT_READY,
    UBX_RESULT_ACK,
    UBX_RESULT_ERROR
} ubx_result_t;

// --- Callback types ---
typedef void (*ubx_pvt_callback_t)(const UBX_NAV_PVT_data_t*);
typedef void (*ubx_ack_callback_t)(uint8_t ack_id);

// --- Interface ---
void ubx_set_pvt_callback(ubx_pvt_callback_t cb);
void ubx_set_ack_callback(ubx_ack_callback_t cb);
ubx_result_t ubx_parse_byte(uint8_t byte);

#ifdef __cplusplus
}
#endif
