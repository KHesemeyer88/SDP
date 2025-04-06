// ubx_parser.cpp
#include "ubx_parser.h"
#include <string.h> // for memcpy

// Parser states
typedef enum {
    UBX_STATE_SYNC1,
    UBX_STATE_SYNC2,
    UBX_STATE_CLASS,
    UBX_STATE_ID,
    UBX_STATE_LEN1,
    UBX_STATE_LEN2,
    UBX_STATE_PAYLOAD,
    UBX_STATE_CK_A,
    UBX_STATE_CK_B
} ubx_state_t;

static ubx_state_t state = UBX_STATE_SYNC1;
static uint8_t msg_class = 0;
static uint8_t msg_id = 0;
static uint16_t payload_len = 0;
static uint8_t payload[UBX_NAV_PVT_LEN];
static uint16_t payload_index = 0;
static uint8_t ck_a = 0, ck_b = 0;
static uint8_t recv_ck_a = 0, recv_ck_b = 0;

static ubx_pvt_callback_t user_callback = 0;

void ubx_set_pvt_callback(ubx_pvt_callback_t cb) {
    user_callback = cb;
}

static void reset_parser() {
    state = UBX_STATE_SYNC1;
    ck_a = ck_b = 0;
    payload_index = 0;
}

ubx_result_t ubx_parse_byte(uint8_t byte) {
    switch (state) {
        case UBX_STATE_SYNC1:
            if (byte == UBX_SYNC1) {
                state = UBX_STATE_SYNC2;
            }
            break;
        case UBX_STATE_SYNC2:
            if (byte == UBX_SYNC2) {
                state = UBX_STATE_CLASS;
            } else {
                state = UBX_STATE_SYNC1;
            }
            break;
        case UBX_STATE_CLASS:
            msg_class = byte;
            ck_a = byte;
            ck_b = ck_a;
            state = UBX_STATE_ID;
            break;
        case UBX_STATE_ID:
            msg_id = byte;
            ck_a += byte;
            ck_b += ck_a;
            state = UBX_STATE_LEN1;
            break;
        case UBX_STATE_LEN1:
            payload_len = byte;
            ck_a += byte;
            ck_b += ck_a;
            state = UBX_STATE_LEN2;
            break;
        case UBX_STATE_LEN2:
            payload_len |= (byte << 8);
            ck_a += byte;
            ck_b += ck_a;
            if (payload_len != UBX_NAV_PVT_LEN) {
                reset_parser();
                return UBX_RESULT_ERROR;
            }
            state = UBX_STATE_PAYLOAD;
            payload_index = 0;
            break;
        case UBX_STATE_PAYLOAD:
            if (payload_index < UBX_NAV_PVT_LEN) {
                payload[payload_index++] = byte;
                ck_a += byte;
                ck_b += ck_a;
                if (payload_index == UBX_NAV_PVT_LEN) {
                    state = UBX_STATE_CK_A;
                }
            }
            break;
        case UBX_STATE_CK_A:
            recv_ck_a = byte;
            state = UBX_STATE_CK_B;
            break;
        case UBX_STATE_CK_B:
            recv_ck_b = byte;
            if (recv_ck_a == ck_a && recv_ck_b == ck_b) {
                if (msg_class == UBX_CLASS_NAV && msg_id == UBX_ID_NAV_PVT && user_callback) {
                    UBX_NAV_PVT_data_t pvt;
                    memcpy(&pvt.iTOW, &payload[0], 4);
                    memcpy(&pvt.lon, &payload[4], 4);
                    memcpy(&pvt.lat, &payload[8], 4);
                    memcpy(&pvt.height, &payload[12], 4);
                    memcpy(&pvt.hMSL, &payload[16], 4);
                    memcpy(&pvt.hAcc, &payload[24], 4);
                    pvt.fixType = payload[20];
                    pvt.flags = payload[21];
                    pvt.numSV = payload[23];
                    pvt.year = payload[28] | (payload[29] << 8);
                    pvt.month = payload[30];
                    pvt.day = payload[31];
                    pvt.hour = payload[32];
                    pvt.min = payload[33];
                    pvt.sec = payload[34];
                    user_callback(&pvt);
                }
                reset_parser();
                return UBX_RESULT_PVT_READY;
            } else {
                reset_parser();
                return UBX_RESULT_ERROR;
            }
            break;
    }
    return UBX_RESULT_NONE;
}
