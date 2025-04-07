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
const unsigned long maxTimeBeforeHangup_ms = 10000UL;

static char lastGGA[128] = {0};
static bool systemTimeSet = false;

volatile int lastAckResult = -1;

SemaphoreHandle_t gnssSpiMutex = NULL;

static volatile bool valgetReady = false;
static volatile ubx_valget_u8_result_t lastValget = {0};

void send_valget_u8(uint32_t key);

static void handleVALGET(const ubx_valget_u8_result_t* res) {
    if (res && res->valid) {
        lastValget.key = res->key;
        lastValget.val = res->val;
        lastValget.valid = res->valid;
        valgetReady = true;
    }
}

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

bool send_valset_u8_blocking(uint32_t key, uint8_t val) {
    lastAckResult = -1;
    // Build VALSET packet
    uint8_t payload[11] = {
        0x00, 0x00, 0x03, 0x00, 0x00, 0x00, // RAM | BBR | FLASH
        (uint8_t)(key), (uint8_t)(key >> 8),
        (uint8_t)(key >> 16), (uint8_t)(key >> 24),
        val
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

    extern volatile int lastAckResult;
    lastAckResult = -1;

    // Send packet
    ubx_write_packet(packet, sizeof(packet));

    // Wait for ACK response
    unsigned long start = millis();
    while (millis() - start < 200) {
        processGNSSInput();
        if (lastAckResult != -1) break;
        delay(5);  // Let SPI rest; helps if ACK comes a few ms later
    }

    if (lastAckResult == 1) {
        LOG_DEBUG("VALSET 0x%08lX accepted (ACK)", key);
        return true;
    } else if (lastAckResult == 0) {
        LOG_ERROR("VALSET 0x%08lX rejected (NAK)", key);
        return false;
    } else {
        LOG_ERROR("VALSET 0x%08lX timed out (no ACK)", key);
        return false;
    }
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
    delay(3000); // Give GNSS module time to stabilize

    // Register UBX callbacks
    ubx_set_pvt_callback(handlePVT);
    ubx_set_ack_callback(handleACK);
    ubx_set_valget_callback(handleVALGET);

    delay(50);
    // send_valset_u8_blocking(0x107A0001, 1); // known to work from u-center
    // delay(50);

    // //send_valset_u8_blocking(0x10540001, 1);  // CFG-SPIOUTPROT-UBX
    // // delay(100);
    // // uint8_t val;
    // // get_val_u8(0x10540001, &val);
    // // LOG_ERROR("Post-VALSET readback: 0x%02X", val);
    // uint8_t val;
    // get_val_u8(0x107A0001, &val);
    // LOG_ERROR("Readback: 0x%02X", val);

    //TEST:
    send_valset_u8_blocking(0x10540001, 1);  // CFG-SPIOUTPROT-UBX
    vTaskDelay(pdMS_TO_TICKS(20));
    uint8_t val = 0xFF;
    if (get_val_u8(0x10540001, &val)) {
        LOG_ERROR("VALGET readback: 0x%02X", val);
    } else {
        LOG_ERROR("VALGET failed for CFG-SPIOUTPROT-UBX");
    }

    struct {
        uint32_t key;
        uint8_t  val;
        const char* label;
    } gnssInitConfig[] = {
        { 0x10540002, 0, "CFG-SPIOUTPROT-NMEA" },
        { 0x10540001, 1, "CFG-SPIOUTPROT-UBX" },
        { 0x10740001, 1, "CFG-SPIINPROT-UBX" },
        { 0x10740004, 1, "CFG-SPIINPROT-RTCM" },
        { 0x20910007, 1, "CFG-MSGOUT-UBX_NAV_PVT_SPI" },
        { 0x30210001, 100, "CFG-RATE-MEAS" },
        { 0x30210002, 1, "CFG-RATE-NAV" }
    };
    
    for (size_t i = 0; i < sizeof(gnssInitConfig) / sizeof(gnssInitConfig[0]); ++i) {
        const auto& cfg = gnssInitConfig[i];
        LOG_DEBUG("VALSET: %s -> %d", cfg.label, cfg.val);
        if (!send_valset_u8_blocking(cfg.key, cfg.val)) {
            LOG_ERROR("VALSET failed for %s (key 0x%08lX)", cfg.label, cfg.key);
        } else {
            LOG_DEBUG("VALSET success: %s", cfg.label);
        }
    }    
    
    struct {
        uint32_t key;
        const char* label;
    } gnssReadbackKeys[] = {
        { 0x10540002, "CFG-SPIOUTPROT-NMEA" },
        { 0x10540001, "CFG-SPIOUTPROT-UBX" },
        { 0x10740001, "CFG-SPIINPROT-UBX" },
        { 0x10740004, "CFG-SPIINPROT-RTCM" },
        { 0x20910007, "CFG-MSGOUT-UBX_NAV_PVT_SPI" },
        { 0x30210001, "CFG-RATE-MEAS" },
        { 0x30210002, "CFG-RATE-NAV" }
    };
    
    for (size_t i = 0; i < sizeof(gnssReadbackKeys) / sizeof(gnssReadbackKeys[0]); ++i) {
        const auto& item = gnssReadbackKeys[i];
        uint8_t val;
        if (get_val_u8(item.key, &val)) {
            LOG_ERROR("VALGET %s (0x%08lX): 0x%02X", item.label, item.key, val);
        } else {
            LOG_ERROR("VALGET FAILED for %s (0x%08lX)", item.label, item.key);
        }
    }
    
    // Optional: send one poll to confirm early communication
    LOG_DEBUG("Polling NAV-PVT once to verify GNSS response");
    poll_nav_pvt();
    delay(50);
    processGNSSInput();

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

    if (dataAvailable && xSemaphoreTake(rtcmRingMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        while (ntripClient.available()) {
            uint8_t b = ntripClient.read();
            size_t nextHead = (ringHead + 1) % RTCM_RING_SIZE;
            if (nextHead == ringTail) {
                ringTail = (ringTail + 1) % RTCM_RING_SIZE; // Overwrite oldest
            }
            rtcmRing[ringHead] = b;
            ringHead = nextHead;
            bytesRead++;
        }

        xSemaphoreGive(rtcmRingMutex);
        xSemaphoreGive(ntripClientMutex);
        lastInjectedRTCM_ms = millis();

        if (bytesRead > 0) {
            LOG_DEBUG("RTCM received: %u bytes added to ring buffer", bytesRead);
        }
    }

    // correctionAge = millis() - lastReceivedRTCM_ms;
    // if (correctionAge < 5000) rtcmCorrectionStatus = CORR_FRESH;
    // else if (correctionAge < 30000) rtcmCorrectionStatus = CORR_STALE;
    // else rtcmCorrectionStatus = CORR_NONE;

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
            // Copy last known fix into a local PVT
            currentPVT.lat = gnssData.latitude * 1e7;
            currentPVT.lon = gnssData.longitude * 1e7;
            currentPVT.fixType = gnssData.fixType;
            currentPVT.hAcc = gnssData.hAcc * 10;
            currentPVT.hMSL = gnssData.latitude != 0 ? 100000 : 0; // Default fake value for altitude
            currentPVT.flags = (gnssData.carrSoln & 0x03) << 6;
            currentPVT.hour = 12;  // dummy
            currentPVT.min = 0;
            currentPVT.sec = 0;
            currentPVT.numSV = 10;
            xSemaphoreGive(gnssMutex);
        } else {
            vTaskDelay(interval);
            continue;
        }

        // Generate GGA string
        char gga[128];
        if (generateGGA(&currentPVT, gga, sizeof(gga))) {
            LOG_DEBUG("GGA Task: %s", gga);
            
            // Protect SPI access with mutex
            if (xSemaphoreTake(gnssSpiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                GNSSSPI.beginTransaction(SPISettings(UBX_SPI_FREQ, MSBFIRST, SPI_MODE0));
                digitalWrite(UBX_CS, LOW);
                delayMicroseconds(1);
                for (const char* p = gga; *p; ++p)
                    GNSSSPI.transfer(*p);
                digitalWrite(UBX_CS, HIGH);
                GNSSSPI.endTransaction();
                xSemaphoreGive(gnssSpiMutex);
            } else {
                LOG_ERROR("Failed to get SPI mutex in GGATask");
            }

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
        
        correctionAge = millis() - lastInjectedRTCM_ms;    
        if (correctionAge < 5000) rtcmCorrectionStatus = CORR_FRESH;
        else if (correctionAge < 30000) rtcmCorrectionStatus = CORR_STALE;
        else rtcmCorrectionStatus = CORR_NONE;
        
        vTaskDelayUntil(&lastWake, interval);
    }
}

bool get_val_u8(uint32_t key, uint8_t* out) {
    if (!out) return false;

    valgetReady = false;
    lastValget.valid = false;

    send_valget_u8(key);  // Just send request

    unsigned long start = millis();
    while (!valgetReady && millis() - start < 200) {
        processGNSSInput();  // parse incoming UBX data
        delay(5);            // avoid locking SPI constantly
    }

    if (valgetReady && lastValget.key == key && lastValget.valid) {
        *out = lastValget.val;
        return true;
    }

    return false;
}

void send_valget_u8(uint32_t key) {
    uint8_t payload[8] = {
        0x00, 0x00, 0x03, 0x00,  // version, layers = RAM+BBR+FLASH, position = 0
        (uint8_t)(key), (uint8_t)(key >> 8),
        (uint8_t)(key >> 16), (uint8_t)(key >> 24)
    };

    uint8_t packet[8 + sizeof(payload)];
    packet[0] = 0xB5; packet[1] = 0x62;
    packet[2] = 0x06; packet[3] = 0x8B;  // VALGET
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
}

