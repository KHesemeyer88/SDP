// gnss.cpp - rewritten to use minimal UBX parser + GGA

#include "gnss.h"
#include "ubx_parser.h"
#include "logging.h"
#include "websocket_handler.h"
#include <SPI.h>
#include <sys/time.h>
#include <stdio.h>
#include <string.h>

static unsigned long gnssByteCount = 0;
static unsigned long lastPVTTime = 0;

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
        lastPVTTime = millis();
        LOG_DEBUG("handlePVT() lat: %ld lon: %ld fix: %d", pvt->lat, pvt->lon, pvt->fixType);

        gnssData.latitude = pvt->lat / 1e7;
        gnssData.longitude = pvt->lon / 1e7;
        gnssData.speed = 0.0f;
        gnssData.fixType = pvt->fixType;
        gnssData.heading = 0.0f;
        gnssData.carrSoln = pvt->flags;
        gnssData.hAcc = pvt->hAcc / 10.0;
        gnssData.newDataAvailable = true;
        gnssData.gnssFixTime = millis();

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
            LOG_DEBUG("RTCM read: %u bytes", count);
        }
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
    LOG_DEBUG("correctionAge = %lu ms, status = %d", correctionAge, rtcmCorrectionStatus);

    if (correctionAge < 5000) rtcmCorrectionStatus = CORR_FRESH;
    else if (correctionAge < 30000) rtcmCorrectionStatus = CORR_STALE;
    else rtcmCorrectionStatus = CORR_NONE;

    return clientConnected;
}

void processGNSSInput() {
    GNSSSPI.beginTransaction(SPISettings(UBX_SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(UBX_CS, LOW);
    delayMicroseconds(1);
    for (int i = 0; i < 200; ++i) {
        uint8_t b = GNSSSPI.transfer(0xFF);
        ubx_parse_byte(b);
    }
    for (int i = 0; i < 200; ++i) {
        uint8_t b = GNSSSPI.transfer(0xFF);
        ubx_parse_byte(b);
        gnssByteCount++;
    }
    LOG_DEBUG("processGNSSInput read %lu bytes", gnssByteCount);
    
    digitalWrite(UBX_CS, HIGH);
    GNSSSPI.endTransaction();
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