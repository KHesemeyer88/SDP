// gnss.cpp - clean rewrite with improved UBX integration

#include "gnss.h"
#include "ubx_parser.h"
#include "logging.h"
#include "websocket_handler.h"
#include <SPI.h>
#include <sys/time.h>
#include <stdio.h>
#include <string.h>
#include "mbedtls/base64.h"

#define UBX_CS NAV_CS_PIN
#define UBX_SPI_FREQ NAV_SPI_FREQUENCY
#define RTCM_RING_SIZE 2048

static SPIClass GNSSSPI(HSPI);
TaskHandle_t gnssTaskHandle = NULL;
volatile GNSSData gnssData = {0};
volatile UBX_NAV_PVT_data_t lastValidPVT = {0};
SemaphoreHandle_t gnssMutex = NULL;
SemaphoreHandle_t rtcmRingMutex = NULL; 

uint8_t rtcmRing[RTCM_RING_SIZE];
volatile size_t ringHead = 0;
volatile size_t ringTail = 0;

extern WiFiClient ntripClient;
extern SemaphoreHandle_t ntripClientMutex;
volatile CorrectionStatus rtcmCorrectionStatus = CORR_NONE;
unsigned long correctionAge = 0;
unsigned long lastInjectedRTCM_ms = 0;
unsigned long lastReceivedRTCM_ms = 0;
const unsigned long maxTimeBeforeHangup_ms = 10000UL;

static char lastGGA[128] = {0};
static bool systemTimeSet = false;

volatile int lastAckResult = -1;

SemaphoreHandle_t gnssSpiMutex = NULL;

// --- Internal SPI helpers ---
static void ubx_write_packet(const uint8_t* data, size_t len) {
    if (xSemaphoreTake(gnssSpiMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        LOG_ERROR("Failed to get SPI mutex for ubx_write_packet");
        return;
    }
    
    GNSSSPI.beginTransaction(SPISettings(UBX_SPI_FREQ, MSBFIRST, SPI_MODE0));
    LOG_DEBUG("CS LOW, starting SPI write");
    digitalWrite(UBX_CS, LOW);
    delayMicroseconds(1);
    for (size_t i = 0; i < len; ++i)
        GNSSSPI.transfer(data[i]);
    digitalWrite(UBX_CS, HIGH);
    LOG_DEBUG("SPI write complete, CS HIGH");
    GNSSSPI.endTransaction();
    
    xSemaphoreGive(gnssSpiMutex);
}

static void poll_nav_pvt() {
    const uint8_t cmd[] = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19};
    ubx_write_packet(cmd, sizeof(cmd));
    LOG_DEBUG("NAV-PVT poll sent");
}

// --- GNSS Callbacks ---
static void handlePVT(const UBX_NAV_PVT_data_t* pvt) {
    unsigned long tCbStart = millis();
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

        memcpy((void*)&lastValidPVT, (const void*)pvt, sizeof(UBX_NAV_PVT_data_t));

        LOG_DEBUG("handlePVT() lat: %.7f lon: %.7f fix: %d carrSoln: %d", 
          gnssData.latitude, gnssData.longitude, gnssData.fixType, gnssData.carrSoln);

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
            LOG_ERROR("------------------------------------------");
            LOG_ERROR("GNSS SYS TIME: %04d-%02d-%02d %02d:%02d (UTC)",
                      pvt->year, pvt->month, pvt->day, pvt->hour, pvt->min);
            LOG_ERROR("------------------------------------------");
        }
        xSemaphoreGive(gnssMutex);
    }
    generateGGA(pvt, lastGGA, sizeof(lastGGA));
    unsigned long tCbElapsed = millis() - tCbStart;
    if (tCbElapsed > 0) {
        LOG_ERROR("GNSS checkCallbacks time, %lu", tCbElapsed);
    }
}

static void handleACK(uint8_t ack_id) {
    if (ack_id == 0x01) {
        lastAckResult = 1;
        LOG_ERROR("Received UBX-ACK-ACK");
    } else {
        lastAckResult = 0;
        LOG_ERROR("Received UBX-ACK-NAK");
    }
}


// --- Initialization ---
bool initializeGNSS() {
    LOG_DEBUG("initializeGNSS() start");

    // Configure SPI pins
    pinMode(UBX_CS, OUTPUT);
    digitalWrite(UBX_CS, HIGH);
    pinMode(NAV_RST_PIN, OUTPUT);
    digitalWrite(NAV_RST_PIN, HIGH);

    // Start SPI interface
    LOG_DEBUG("Calling GNSSSPI.begin()");
    GNSSSPI.begin(NAV_SCK_PIN, NAV_MISO_PIN, NAV_MOSI_PIN, UBX_CS);
    LOG_ERROR("Waiting for first fix before sending VALSET...");

    unsigned long start = millis();
    while (millis() - start < 10000) {
        processGNSSInput();
        if (xSemaphoreTake(gnssMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            bool ready = gnssData.fixType >= 3;
            xSemaphoreGive(gnssMutex);
            if (ready) break;
        }
        delay(100);
    }    

    // Register UBX callbacks
    ubx_set_pvt_callback(handlePVT);
    ubx_set_ack_callback(handleACK);

    delay(50);

    LOG_DEBUG("GNSS initialization complete");
    return true;
}

void processGNSSInput() {
    LOG_DEBUG("Entering processGNSSInput");
    
    if (xSemaphoreTake(gnssSpiMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        LOG_ERROR("Failed to get SPI mutex in processGNSSInput");
        return;
    }
    
    unsigned long tStart = millis();
    GNSSSPI.beginTransaction(SPISettings(UBX_SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(UBX_CS, LOW);
    delayMicroseconds(1);

    uint8_t buf[200];
    for (int i = 0; i < sizeof(buf); ++i) {
        buf[i] = GNSSSPI.transfer(0xFF);
        ubx_parse_byte(buf[i]);
    }

    digitalWrite(UBX_CS, HIGH);
    GNSSSPI.endTransaction();
    
    xSemaphoreGive(gnssSpiMutex);
    
    unsigned long tElapsed = millis() - tStart;
    if (tElapsed > 0) {
        LOG_ERROR("GNSS checkUblox time, %lu", tElapsed);
    }


    // for (int i = 0; i < 64; i += 8) {
    //     LOG_DEBUG("SPI bytes: %02X %02X %02X %02X %02X %02X %02X %02X",
    //         buf[i], buf[i+1], buf[i+2], buf[i+3],
    //         buf[i+4], buf[i+5], buf[i+6], buf[i+7]);
    // }
}

bool processRTKConnection() {
    LOG_DEBUG("Entering processRTKConnection, about to set clientConnected false");
    bool clientConnected = false;
    bool dataAvailable = false;

    if (xSemaphoreTake(ntripClientMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        clientConnected = ntripClient.connected();
        dataAvailable = clientConnected && ntripClient.available();
        if (!dataAvailable)
            xSemaphoreGive(ntripClientMutex);
    }
    LOG_DEBUG("processRTKConnection: connected=%d, available=%d",
        clientConnected, dataAvailable);

    size_t bytesRead = 0;
    
    if (dataAvailable && xSemaphoreTake(gnssSpiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        GNSSSPI.beginTransaction(SPISettings(UBX_SPI_FREQ, MSBFIRST, SPI_MODE0));
        digitalWrite(UBX_CS, LOW);
        delayMicroseconds(1);
        while (ntripClient.available()) {
            uint8_t b = ntripClient.read();
            GNSSSPI.transfer(b);
        }
        digitalWrite(UBX_CS, HIGH);
        GNSSSPI.endTransaction();
        xSemaphoreGive(gnssSpiMutex);
        xSemaphoreGive(ntripClientMutex);
        lastReceivedRTCM_ms = millis();
    }
    
    // if (dataAvailable && xSemaphoreTake(rtcmRingMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    //     while (ntripClient.available()) {
    //         uint8_t b = ntripClient.read();
    //         size_t nextHead = (ringHead + 1) % RTCM_RING_SIZE;
    //         if (nextHead == ringTail) {
    //             ringTail = (ringTail + 1) % RTCM_RING_SIZE; // Overwrite oldest
    //         }
    //         rtcmRing[ringHead] = b;
    //         ringHead = nextHead;
    //         bytesRead++;
    //     }

    //     xSemaphoreGive(rtcmRingMutex);
    //     xSemaphoreGive(ntripClientMutex);
    //     lastReceivedRTCM_ms = millis();

    //     if (bytesRead > 0) {
    //         LOG_DEBUG("RTCM received: %u bytes added to ring buffer", bytesRead);
    //     }
    // }

    correctionAge = millis() - lastReceivedRTCM_ms;
    if (correctionAge < 5000) rtcmCorrectionStatus = CORR_FRESH;
    else if (correctionAge < 30000) rtcmCorrectionStatus = CORR_STALE;
    else rtcmCorrectionStatus = CORR_NONE;

    return clientConnected;
}

bool connectToNTRIP() {
    LOG_DEBUG("Entering connectToNTRIP");
    if (xSemaphoreTake(ntripClientMutex, pdMS_TO_TICKS(500)) != pdTRUE)
        return false;

    if (ntripClient.connected()) {
        xSemaphoreGive(ntripClientMutex);
        return true;
    }

    if (!ntripClient.connect(casterHost, casterPort)) {
        LOG_ERROR("Failed to connect to caster");
        xSemaphoreGive(ntripClientMutex);
        return false;
    }

    char auth[128], encoded[200];
    snprintf(auth, sizeof(auth), "%s:%s", casterUser, casterUserPW);
    encodeBase64(auth, encoded, sizeof(encoded));

    char request[512];
    snprintf(request, sizeof(request),
        "GET /%s HTTP/1.0\r\n"
        "User-Agent: NTRIP u-blox Client\r\n"
        "Accept: */*\r\n"
        "Connection: close\r\n"
        "Authorization: Basic %s\r\n"
        "\r\n",
        mountPoint, encoded
    );

    ntripClient.write(request, strlen(request));
    LOG_DEBUG("Sent NTRIP request to %s:%u", casterHost, casterPort);
    delay(500);  // Give time for HTTP response

    char response[256] = {0};
    int idx = 0;
    while (ntripClient.available() && idx < sizeof(response) - 1) {
        response[idx++] = ntripClient.read();
    }
    response[idx] = '\0';
    LOG_DEBUG("NTRIP HTTP response:\n%s", response);

    xSemaphoreGive(ntripClientMutex);

    LOG_DEBUG("NTRIP socket state: connected=%d, available=%d", 
        ntripClient.connected(), ntripClient.available());

    return true;
}

// --- FreeRTOS task ---
void GNSSTask(void *pvParameters) {
    LOG_DEBUG("GNSSTask started");
    initializeGNSS();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / NAV_UPDATE_FREQUENCY);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    LOG_DEBUG("GNSSTask about to enter main loop");
    while (true) {
        LOG_DEBUG("GNSSTask inside main loop");
        const unsigned long maxCorrectionAgeBeforeReconnect = 1000; 
        static bool forceReconnect = true;

        if (forceReconnect || correctionAge > maxCorrectionAgeBeforeReconnect) {
            LOG_DEBUG("NTRIP reconnect triggered (correctionAge = %lu ms)", correctionAge);
            connectToNTRIP();
            forceReconnect = false;
        }

        bool gotRTCM = processRTKConnection();
        if (!gotRTCM) {
            // could log or increment diagnostics counter here
        }
    
        processGNSSInput();
        processRTKConnection();
        LOG_DEBUG("GNSSTask stack high water mark: %u", uxTaskGetStackHighWaterMark(NULL));
    
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

    static unsigned long lastLogTime = 0;
    unsigned long now = millis();
    if (now - lastLogTime > 3000) {
        LOG_DEBUG("generateGGA(): %s", out);
        lastLogTime = now;
    }

    return true;
}

void GGATask(void *pvParameters) {
    LOG_DEBUG("GGATask Start");
    const TickType_t interval = pdMS_TO_TICKS(1000); // 1 Hz
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true) {
        UBX_NAV_PVT_data_t currentPVT;

        if (xSemaphoreTake(gnssMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memcpy(&currentPVT, (const void*)&lastValidPVT, sizeof(UBX_NAV_PVT_data_t));
            xSemaphoreGive(gnssMutex);
        } else {
            vTaskDelay(interval);
            continue;
        }

        // Generate GGA string
        char gga[128];
        if (generateGGA(&currentPVT, gga, sizeof(gga))) {
            LOG_DEBUG("GGA Task: %s", gga);
            // GGA string generated and sent to NTRIP caster only.
            // No need to transmit it over SPI to the GNSS module.
            //MAY NEED TO REVISIT THIS IF RTK NEVER WORKS
            // Also forward GGA to the NTRIP caster
            if (xSemaphoreTake(ntripClientMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (ntripClient.connected()) {
                    ntripClient.print(gga);
                    LOG_DEBUG("GGA forwarded to caster: %s", gga);
                }
                xSemaphoreGive(ntripClientMutex);
            }
        }

        vTaskDelayUntil(&lastWakeTime, interval);
    }
}

bool encodeBase64(const char* input, char* output, size_t outputSize) {
    size_t olen = 0;
    int ret = mbedtls_base64_encode(
        (unsigned char*)output, outputSize,
        &olen,
        (const unsigned char*)input,
        strlen(input)
    );
    return (ret == 0);
}

void RTCMInjectionTask(void *pvParameters) {
    LOG_DEBUG("RTCMInjectionTask start");
    const TickType_t interval = pdMS_TO_TICKS(500);
    TickType_t lastWake = xTaskGetTickCount();

    uint8_t chunk[256];

    while (true) {
        
        LOG_DEBUG("RTCMInjectionTask: ringHead=%u, ringTail=%u", ringHead, ringTail);
        size_t count = 0;

        if (xSemaphoreTake(rtcmRingMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            while (ringTail != ringHead && count < sizeof(chunk)) {
                chunk[count++] = rtcmRing[ringTail];
                ringTail = (ringTail + 1) % RTCM_RING_SIZE;
            }
            xSemaphoreGive(rtcmRingMutex);
        }

        if (count > 0) {
            LOG_DEBUG("Injecting RTCM chunk (%u bytes)", count);
            unsigned long tPushStart = millis();
            
            // Protect SPI access with mutex
            if (xSemaphoreTake(gnssSpiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                GNSSSPI.beginTransaction(SPISettings(UBX_SPI_FREQ, MSBFIRST, SPI_MODE0));
                digitalWrite(UBX_CS, LOW);
                delayMicroseconds(1);
                for (size_t i = 0; i < count; ++i)
                    GNSSSPI.transfer(chunk[i]);
                digitalWrite(UBX_CS, HIGH);
                GNSSSPI.endTransaction();
                xSemaphoreGive(gnssSpiMutex);
                
                unsigned long tPushElapsed = millis() - tPushStart;
                if (tPushElapsed > 0) {
                    LOG_ERROR("pushRawData time, %lu", tPushElapsed);
                }
                
                lastInjectedRTCM_ms = millis(); // rename if you want
            } else {
                LOG_ERROR("Failed to get SPI mutex in RTCMInjectionTask");
            }
        }
        
        // correctionAge = millis() - lastInjectedRTCM_ms;    
        // if (correctionAge < 5000) rtcmCorrectionStatus = CORR_FRESH;
        // else if (correctionAge < 30000) rtcmCorrectionStatus = CORR_STALE;
        // else rtcmCorrectionStatus = CORR_NONE;
        
        vTaskDelayUntil(&lastWake, interval);
    }
}