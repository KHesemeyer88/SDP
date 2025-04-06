// gnss.cpp - clean rewrite with improved UBX integration

#include "gnss.h"
#include "ubx_parser.h"
#include "logging.h"
#include "websocket_handler.h"
#include <SPI.h>
#include <sys/time.h>
#include <stdio.h>
#include <string.h>

#define UBX_CS NAV_CS_PIN
#define UBX_SPI_FREQ NAV_SPI_FREQUENCY

static SPIClass GNSSSPI(HSPI);
TaskHandle_t gnssTaskHandle = NULL;
volatile GNSSData gnssData = {0};
SemaphoreHandle_t gnssMutex = NULL;

extern WiFiClient ntripClient;
extern SemaphoreHandle_t ntripClientMutex;
volatile CorrectionStatus rtcmCorrectionStatus = CORR_NONE;
unsigned long correctionAge = 0;
unsigned long lastReceivedRTCM_ms = 0;
const unsigned long maxTimeBeforeHangup_ms = 10000UL;

static char lastGGA[128] = {0};
static bool systemTimeSet = false;

// --- Internal SPI helpers ---
static void ubx_write_packet(const uint8_t* data, size_t len) {
    GNSSSPI.beginTransaction(SPISettings(UBX_SPI_FREQ, MSBFIRST, SPI_MODE0));
    LOG_DEBUG("CS LOW, starting SPI write");
    digitalWrite(UBX_CS, LOW);
    delayMicroseconds(1);
    for (size_t i = 0; i < len; ++i)
        GNSSSPI.transfer(data[i]);
    digitalWrite(UBX_CS, HIGH);
    LOG_DEBUG("SPI write complete, CS HIGH");
    GNSSSPI.endTransaction();
}

static void poll_nav_pvt() {
    const uint8_t cmd[] = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19};
    ubx_write_packet(cmd, sizeof(cmd));
    LOG_DEBUG("NAV-PVT poll sent");
}

static void send_valset_u8(uint32_t key, uint8_t val) {
    uint8_t payload[11] = {
        0x00, 0x00, 0x07, 0x00, 0x00, 0x00,  // <- enable RAM | BBR | FLASH    
        (uint8_t)(key), (uint8_t)(key >> 8), (uint8_t)(key >> 16), (uint8_t)(key >> 24), val
    };
    uint8_t packet[8 + sizeof(payload)];
    packet[0] = 0xB5; packet[1] = 0x62;
    packet[2] = 0x06; packet[3] = 0x8A;
    packet[4] = sizeof(payload) & 0xFF;
    packet[5] = (sizeof(payload) >> 8) & 0xFF;
    memcpy(&packet[6], payload, sizeof(payload));

    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < sizeof(packet) - 2; ++i) {
        ck_a += packet[i];
        ck_b += ck_a;
    }
    packet[sizeof(packet) - 2] = ck_a;
    packet[sizeof(packet) - 1] = ck_b;

    ubx_write_packet(packet, sizeof(packet));
    LOG_DEBUG("VALSET sent, key: 0x%08lX val: %d", key, val);
}

// --- GNSS Callbacks ---
static void handlePVT(const UBX_NAV_PVT_data_t* pvt) {
    if (xSemaphoreTake(gnssMutex, 0) == pdTRUE) {
        gnssData.latitude = pvt->lat / 1e7;
        gnssData.longitude = pvt->lon / 1e7;
        gnssData.speed = 0.0f;
        gnssData.fixType = pvt->fixType;
        gnssData.heading = 0.0f;
        gnssData.carrSoln = (pvt->flags >> 6) & 0x03;
        gnssData.hAcc = pvt->hAcc / 10.0;
        gnssData.newDataAvailable = true;
        gnssData.gnssFixTime = millis();

        LOG_DEBUG("handlePVT() lat: %ld lon: %ld fix: %d", pvt->lat, pvt->lon, pvt->fixType);

        if (!systemTimeSet && pvt->fixType >= 3) {
            struct tm timeinfo = {0};
            timeinfo.tm_year = pvt->year - 1900;
            timeinfo.tm_mon = pvt->month - 1;
            timeinfo.tm_mday = pvt->day;
            timeinfo.tm_hour = pvt->hour;
            timeinfo.tm_min = pvt->min;
            timeinfo.tm_sec = pvt->sec;
            time_t epoch = mktime(&timeinfo);
            struct timeval tv = {.tv_sec = epoch, .tv_usec = 0};
            settimeofday(&tv, NULL);
            systemTimeSet = true;
        }
        xSemaphoreGive(gnssMutex);
    }
    generateGGA(pvt, lastGGA, sizeof(lastGGA));
}

static void handleACK(uint8_t ack_id) {
    if (ack_id == 0x01) {
        LOG_DEBUG("Received UBX-ACK-ACK");
    } else {
        LOG_ERROR("Received UBX-ACK-NAK");
    }
}

// --- Initialization ---
bool initializeGNSS() {
    LOG_DEBUG("initializeGNSS() start");
    pinMode(UBX_CS, OUTPUT);
    digitalWrite(UBX_CS, HIGH);
    pinMode(NAV_RST_PIN, OUTPUT);
    digitalWrite(NAV_RST_PIN, HIGH);

    LOG_DEBUG("Calling GNSSSPI.begin()");
    GNSSSPI.begin(NAV_SCK_PIN, NAV_MISO_PIN, NAV_MOSI_PIN, UBX_CS);
    delay(100);

    ubx_set_pvt_callback(handlePVT);
    ubx_set_ack_callback(handleACK);

    // Configure protocols and message output
    LOG_DEBUG("Sending VALSET: CFG-SPIOUTPROT-UBX");
    send_valset_u8(0x10540001, 1); // CFG-SPIOUTPROT-UBX
    LOG_DEBUG("Sending poll_nav_pvt() 1");
    poll_nav_pvt(); // send dummy request after config
    delay(50);      // wait for a response
    processGNSSInput(); // manually trigger polling cycle

    send_valset_u8(0x10740001, 1); // CFG-SPIINPROT-UBX
    LOG_DEBUG("Sending poll_nav_pvt() 2");
    poll_nav_pvt(); // send dummy request after config
    delay(50);      // wait for a response
    processGNSSInput(); // manually trigger polling cycle

    send_valset_u8(0x10740004, 1); // CFG-SPIINPROT-RTCM
    LOG_DEBUG("Sending poll_nav_pvt() 3");
    poll_nav_pvt(); // send dummy request after config
    delay(50);      // wait for a response
    processGNSSInput(); // manually trigger polling cycle

    send_valset_u8(0x20910007, 1); // CFG-MSGOUT-UBX_NAV_PVT_SPI
    LOG_DEBUG("Sending poll_nav_pvt() 4");
    poll_nav_pvt(); // send dummy request after config
    delay(50);      // wait for a response
    processGNSSInput(); // manually trigger polling cycle

    return true;
}

void processGNSSInput() {
    GNSSSPI.beginTransaction(SPISettings(UBX_SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(UBX_CS, LOW);
    delayMicroseconds(1);

    uint8_t buf[64];
    for (int i = 0; i < 64; ++i) {
        buf[i] = GNSSSPI.transfer(0xFF);
        ubx_parse_byte(buf[i]);
    }

    digitalWrite(UBX_CS, HIGH);
    GNSSSPI.endTransaction();

    for (int i = 0; i < 64; i += 8) {
        LOG_DEBUG("SPI bytes: %02X %02X %02X %02X %02X %02X %02X %02X",
            buf[i], buf[i+1], buf[i+2], buf[i+3],
            buf[i+4], buf[i+5], buf[i+6], buf[i+7]);
    }
}


// // --- Periodic GNSS byte reader ---
// void processGNSSInput() {
//     GNSSSPI.beginTransaction(SPISettings(UBX_SPI_FREQ, MSBFIRST, SPI_MODE0));
//     digitalWrite(UBX_CS, LOW);
//     delayMicroseconds(1);
//     for (int i = 0; i < 200; ++i) {
//         uint8_t b = GNSSSPI.transfer(0xFF);
//         ubx_parse_byte(b);
//     }
//     digitalWrite(UBX_CS, HIGH);
//     GNSSSPI.endTransaction();
// }

bool processRTKConnection() {
    bool clientConnected = false;
    bool dataAvailable = false;

    if (xSemaphoreTake(ntripClientMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        clientConnected = ntripClient.connected();
        dataAvailable = clientConnected && ntripClient.available();
        if (!dataAvailable)
            xSemaphoreGive(ntripClientMutex);
    }

    if (dataAvailable) {
        uint8_t rtcmBuf[1024];
        size_t count = 0;
        while (ntripClient.available() && count < sizeof(rtcmBuf))
            rtcmBuf[count++] = ntripClient.read();

        LOG_DEBUG("RTCM bytes read: %u", count);
        xSemaphoreGive(ntripClientMutex);

        if (count > 0) {
            lastReceivedRTCM_ms = millis();
            GNSSSPI.beginTransaction(SPISettings(UBX_SPI_FREQ, MSBFIRST, SPI_MODE0));
            digitalWrite(UBX_CS, LOW);
            delayMicroseconds(1);
            for (const char* p = lastGGA; *p; ++p) GNSSSPI.transfer(*p);
            for (size_t i = 0; i < count; ++i) GNSSSPI.transfer(rtcmBuf[i]);
            digitalWrite(UBX_CS, HIGH);
            GNSSSPI.endTransaction();
        }
    }

    correctionAge = millis() - lastReceivedRTCM_ms;
    if (correctionAge < 5000) rtcmCorrectionStatus = CORR_FRESH;
    else if (correctionAge < 30000) rtcmCorrectionStatus = CORR_STALE;
    else rtcmCorrectionStatus = CORR_NONE;

    return clientConnected;
}

// --- FreeRTOS task ---
void GNSSTask(void *pvParameters) {
    LOG_DEBUG("GNSSTask started");
    initializeGNSS();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / NAV_UPDATE_FREQUENCY);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        processGNSSInput();
        processRTKConnection();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// --- GGA generation ---
bool generateGGA(const UBX_NAV_PVT_data_t* pvt, char* out, size_t outLen) {
    if (!pvt || !out || outLen < 100) return false;

    uint32_t lat_deg = abs(pvt->lat / 10000000);
    float lat_min = fabs((pvt->lat / 1e7) - lat_deg) * 60.0;
    uint32_t lon_deg = abs(pvt->lon / 10000000);
    float lon_min = fabs((pvt->lon / 1e7) - lon_deg) * 60.0;
    char lat_dir = (pvt->lat >= 0) ? 'N' : 'S';
    char lon_dir = (pvt->lon >= 0) ? 'E' : 'W';

    int fixQuality = (pvt->fixType >= 3) ? ((pvt->flags >> 6) & 0x03 ? 4 : 1) : 0;
    float altitude = pvt->hMSL / 1000.0f;
    int numSV = pvt->numSV;

    snprintf(out, outLen,
        "$GPGGA,%02d%02d%02d,%02u%07.4f,%c,%03u%07.4f,%c,%d,%02d,%.1f,%.1f,M,0.0,M,,",
        pvt->hour, pvt->min, pvt->sec,
        lat_deg, lat_min, lat_dir,
        lon_deg, lon_min, lon_dir,
        fixQuality, numSV, 0.8f, altitude);

    uint8_t checksum = 0;
    for (size_t i = 1; out[i] != '\0'; i++) checksum ^= out[i];
    char cs[6];
    snprintf(cs, sizeof(cs), "*%02X\r\n", checksum);
    strncat(out, cs, outLen - strlen(out) - 1);
    return true;
}
