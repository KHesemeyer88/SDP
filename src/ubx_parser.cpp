#include "ubx_parser.h"
#include <string.h>
#include <stdint.h>
#include "logging.h"

#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62
#define UBX_NAV_PVT_EXPECTED_LEN 92

static ubx_valget_callback_t valget_callback = NULL;
void ubx_set_valget_callback(ubx_valget_callback_t cb) { valget_callback = cb; }

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
static uint8_t payload[256]; // Safe cap
static uint16_t payload_index = 0;
static uint8_t ck_a = 0, ck_b = 0;
static uint8_t recv_ck_a = 0, recv_ck_b = 0;

static ubx_pvt_callback_t pvt_callback = NULL;
static ubx_ack_callback_t ack_callback = NULL;

void ubx_set_pvt_callback(ubx_pvt_callback_t cb) { pvt_callback = cb; }
void ubx_set_ack_callback(ubx_ack_callback_t cb) { ack_callback = cb; }

static void reset_parser() {
    state = UBX_STATE_SYNC1;
    ck_a = ck_b = 0;
    payload_index = 0;
}

ubx_result_t ubx_parse_byte(uint8_t byte) {
    switch (state) {
        case UBX_STATE_SYNC1:
            if (byte == UBX_SYNC1)
                state = UBX_STATE_SYNC2;
            break;

        case UBX_STATE_SYNC2:
            if (byte == UBX_SYNC2)
                state = UBX_STATE_CLASS;
            else
                state = UBX_STATE_SYNC1;
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
            payload_len |= ((uint16_t)byte << 8);
            ck_a += byte;
            ck_b += ck_a;
            if (payload_len > sizeof(payload)) {
                reset_parser();
                return UBX_RESULT_ERROR;
            }
            payload_index = 0;
            state = (payload_len > 0) ? UBX_STATE_PAYLOAD : UBX_STATE_CK_A;
            break;

        case UBX_STATE_PAYLOAD:
            payload[payload_index++] = byte;
            ck_a += byte;
            ck_b += ck_a;
            if (payload_index >= payload_len)
                state = UBX_STATE_CK_A;
            break;

        case UBX_STATE_CK_A:
            recv_ck_a = byte;
            state = UBX_STATE_CK_B;
            break;

        case UBX_STATE_CK_B:
            recv_ck_b = byte;
            if (recv_ck_a == ck_a && recv_ck_b == ck_b) {
                if (msg_class == UBX_CLASS_NAV && msg_id == UBX_ID_NAV_PVT &&
                    payload_len == UBX_NAV_PVT_EXPECTED_LEN && pvt_callback) {

                    UBX_NAV_PVT_data_t pvt;
                    memset(&pvt, 0, sizeof(pvt));

                    memcpy(&pvt.iTOW,    &payload[0],  4);
                    memcpy(&pvt.lon,     &payload[24], 4);
                    memcpy(&pvt.lat,     &payload[28], 4);
                    memcpy(&pvt.height,  &payload[32], 4);
                    memcpy(&pvt.hMSL,    &payload[36], 4);
                    memcpy(&pvt.hAcc,    &payload[40], 4);

                    pvt.fixType = payload[20];
                    pvt.flags   = payload[21];
                    pvt.numSV   = payload[23];

                    pvt.year    = payload[4]  | (payload[5] << 8);
                    pvt.month   = payload[6];
                    pvt.day     = payload[7];
                    pvt.hour    = payload[8];
                    pvt.min     = payload[9];
                    pvt.sec     = payload[10];

                    pvt_callback(&pvt);
                    reset_parser();
                    return UBX_RESULT_PVT_READY;
                }
                else if (msg_class == 0x05 && ack_callback && payload_len == 2) {
                    ack_callback(msg_id); // 0x01 = ACK-ACK, 0x00 = ACK-NAK
                    reset_parser();
                    return UBX_RESULT_ACK;
                }
                else if (msg_class == 0x06 && msg_id == 0x8B && valget_callback) {
                    // We're assuming:
                    //   - layers = 0x07 (RAM+BBR+Flash)
                    //   - position = 0
                    //   - single key (i.e., payload[0..3] = key, payload[4] = val)
                    if (payload_len >= 5) {
                        ubx_valget_u8_result_t result;
                        result.key = payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24);
                        result.val = payload[4];
                        result.valid = true;
                        LOG_ERROR("VALGET response received, payload_len=%d", payload_len);
                        for (int i = 0; i < payload_len && i < 32; i += 8) {
                            LOG_ERROR("VALGET bytes: %02X %02X %02X %02X %02X %02X %02X %02X",
                                payload[i], payload[i+1], payload[i+2], payload[i+3],
                                payload[i+4], payload[i+5], payload[i+6], payload[i+7]);
                        }

                        valget_callback(&result);
                        reset_parser();
                        return UBX_RESULT_VALGET_READY;
                    }
                }
                
            } else {
                reset_parser();
                return UBX_RESULT_ERROR;
            }
            reset_parser();
            break;
    }

    return UBX_RESULT_NONE;
}
