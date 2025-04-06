// gnss.cpp - rewritten to use minimal UBX parser + GGA + config

#include "gnss.h"
#include "ubx_parser.h"
#include "logging.h"
#include "websocket_handler.h"
#include <SPI.h>
#include <sys/time.h>
#include <stdio.h>
#include <string.h>

TaskHandle_t gnssTaskHandle = NULL;
volatile GNSSData gnssData = {0};
SemaphoreHandle_t gnssMutex = NULL;

extern WiFiClient ntripClient;
extern SemaphoreHandle_t ntripClientMutex;
volatile CorrectionStatus rtcmCorrectionStatus = CORR_NONE;
unsigned long correctionAge = 0;
unsigned long lastReceivedRTCM_ms = 0;
const unsigned long maxTimeBeforeHangup_ms = 10000UL;

static SPIClass GNSSSPI(HSPI);
#define UBX_CS NAV_CS_PIN
#define UBX_SPI_FREQ NAV_SPI_FREQUENCY
static bool systemTimeSet = false;

static char lastGGA[128] = {0};

static void storeGGA(const UBX_NAV_PVT_data_t* pvt);

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
    storeGGA(pvt);
}

static void ubx_write_packet(const uint8_t* data, size_t len) {
    GNSSSPI.beginTransaction(SPISettings(UBX_SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(UBX_CS, LOW);
    delayMicroseconds(1);
    for (size_t i = 0; i < len; ++i) {
        GNSSSPI.transfer(data[i]);
    }
    digitalWrite(UBX_CS, HIGH);
    GNSSSPI.endTransaction();
}

static bool gnss_send_cfg_valset_u8(uint32_t key, uint8_t val) {
    uint8_t payload[19];
    payload[0] = 0x00; // version
    payload[1] = 0x00; // reserved
    payload[2] = 0x01; // layer RAM
    payload[3] = 0x00; // transaction
    payload[4] = 0x00; payload[5] = 0x00; // reserved
    payload[6] = key & 0xFF;
    payload[7] = (key >> 8) & 0xFF;
    payload[8] = (key >> 16) & 0xFF;
    payload[9] = (key >> 24) & 0xFF;
    payload[10] = val;

    size_t payloadLen = 11;
    uint8_t packet[32];
    packet[0] = 0xB5;
    packet[1] = 0x62;
    packet[2] = 0x06; // CFG
    packet[3] = 0x8A; // VALSET
    packet[4] = payloadLen & 0xFF;
    packet[5] = (payloadLen >> 8) & 0xFF;
    memcpy(&packet[6], payload, payloadLen);

    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < 6 + payloadLen; ++i) {
        ck_a += packet[i];
        ck_b += ck_a;
    }
    packet[6 + payloadLen] = ck_a;
    packet[7 + payloadLen] = ck_b;

    ubx_write_packet(packet, 8 + payloadLen);
    delay(50); // Give module time to apply
    return true;
}

bool initializeGNSS() {
    LOG_DEBUG("STARTING initializeGNSS (custom parser)");

    pinMode(UBX_CS, OUTPUT);
    digitalWrite(UBX_CS, HIGH);
    pinMode(NAV_RST_PIN, OUTPUT);
    digitalWrite(NAV_RST_PIN, HIGH);

    GNSSSPI.begin(NAV_SCK_PIN, NAV_MISO_PIN, NAV_MOSI_PIN, UBX_CS);
    delay(100);
    GNSSSPI.beginTransaction(SPISettings(UBX_SPI_FREQ, MSBFIRST, SPI_MODE0));
    GNSSSPI.transfer(0xFF);
    GNSSSPI.endTransaction();
    delay(100);

    // Configure protocols and message output
    gnss_send_cfg_valset_u8(0x10540001, 1); // CFG-SPIOUTPROT-UBX
    gnss_send_cfg_valset_u8(0x10740001, 1); // CFG-SPIINPROT-UBX
    gnss_send_cfg_valset_u8(0x10740004, 1); // CFG-SPIINPROT-RTCM
    gnss_send_cfg_valset_u8(0x20910007, 1); // CFG-MSGOUT-UBX_NAV_PVT_SPI

    ubx_set_pvt_callback(handlePVT);
    return true;
}

bool processRTKConnection() {
    bool clientConnected = false;
    bool dataAvailable = false;

    if (xSemaphoreTake(ntripClientMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        clientConnected = ntripClient.connected();
        dataAvailable = clientConnected && ntripClient.available();

        if (!dataAvailable) {
            xSemaphoreGive(ntripClientMutex);
        }
    }

    if (dataAvailable) {
        uint8_t rtcmBuf[512 * 2];
        size_t count = 0;

        while (ntripClient.available() && count < sizeof(rtcmBuf)) {
            rtcmBuf[count++] = ntripClient.read();
        }
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

void processGNSSInput() {
    static unsigned long gnssByteCount = 0;
    GNSSSPI.beginTransaction(SPISettings(UBX_SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(UBX_CS, LOW);
    delayMicroseconds(1);
    for (int i = 0; i < 200; ++i) {
        uint8_t b = GNSSSPI.transfer(0xFF);
        ubx_parse_byte(b);
        gnssByteCount++;
    }
    digitalWrite(UBX_CS, HIGH);
    GNSSSPI.endTransaction();
    if (gnssByteCount % 2000 == 0)
        LOG_DEBUG("GNSS bytes parsed: %lu", gnssByteCount);
}

// --- GGA generation logic ---
static bool generateGGA(const UBX_NAV_PVT_data_t* pvt, char* out, size_t outLen) {
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

    static unsigned long lastGGALog = 0;
    if (millis() - lastGGALog > 1000) {
        lastGGALog = millis();
        LOG_DEBUG("Last GGA: %s", out);
    }

    return true;
}

static void storeGGA(const UBX_NAV_PVT_data_t* pvt) {
    generateGGA(pvt, lastGGA, sizeof(lastGGA));
}

void GNSSTask(void *pvParameters) {
    LOG_DEBUG("GNSSTask started");

    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / NAV_UPDATE_FREQUENCY);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        processGNSSInput();
        processRTKConnection();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
