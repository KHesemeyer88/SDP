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
volatile GNSSData gnssShadow = {0};

volatile UBX_NAV_PVT_data_t lastValidPVT = {0};
SemaphoreHandle_t gnssMutex = NULL;

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

static int mutexWait = 50;

// --- Internal SPI helpers ---
static void ubx_write_packet(const uint8_t* data, size_t len) {
    if (xSemaphoreTake(gnssSpiMutex, pdMS_TO_TICKS(mutexWait)) != pdTRUE) {
        LOG_ERROR("ubx_write_packet gnssSpiMutex timeout");
        return;
    } 
    
    GNSSSPI.beginTransaction(SPISettings(UBX_SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(UBX_CS, LOW);
    delayMicroseconds(1);
    for (size_t i = 0; i < len; ++i)
        GNSSSPI.transfer(data[i]);
    digitalWrite(UBX_CS, HIGH);
    GNSSSPI.endTransaction();
    
    xSemaphoreGive(gnssSpiMutex);
}

static void poll_nav_pvt() {
    const uint8_t cmd[] = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19};
    ubx_write_packet(cmd, sizeof(cmd));
    LOG_NAV("NAV-PVT poll sent");
}

// --- GNSS Callbacks ---
static void handlePVT(const UBX_NAV_PVT_data_t* pvt) {
    unsigned long tCbStart = millis();
    if (xSemaphoreTake(gnssMutex, 0) == pdTRUE) {
        gnssData.latitude = pvt->lat / 1e7;
        gnssData.longitude = pvt->lon / 1e7;
        gnssData.speed = pvt->gSpeed / 1000.0f;
        gnssData.fixType = pvt->fixType;
        gnssData.heading = pvt->headMot / 1e5;
        gnssData.carrSoln = (pvt->flags >> 6) & 0x03;
        gnssData.hAcc = pvt->hAcc / 10.0;
        gnssData.newDataAvailable = true;
        gnssData.gnssFixTime = millis();

        memcpy((void*)&lastValidPVT, (const void*)pvt, sizeof(UBX_NAV_PVT_data_t));

        // LOG_NAV("handlePVT() lat: %.7f lon: %.7f fix: %d carrSoln: %d", 
        //     gnssData.latitude, gnssData.longitude, gnssData.fixType, gnssData.carrSoln);

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
        memcpy((void*)&gnssShadow, (const void*)&gnssData, sizeof(GNSSData));
    } else {
        LOG_ERROR("handlePVT gnssDataMutex timeout");
    }
    //generateGGA(pvt, lastGGA, sizeof(lastGGA));
    unsigned long tCbElapsed = millis() - tCbStart;
    if (tCbElapsed > 0) {
        LOG_NAV("GNSS checkCallbacks time, %lu", tCbElapsed);
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

    unsigned long start = millis();
    while (millis() - start < 10000) {
        processGNSSInput();
        if (xSemaphoreTake(gnssMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            bool ready = gnssData.fixType >= 3;
            xSemaphoreGive(gnssMutex);
            if (ready) break;
        }
        delay(100);
    }    

    // Register UBX callbacks
    ubx_set_pvt_callback(handlePVT);

    delay(50);

    LOG_DEBUG("GNSS initialization complete");
    return true;
}

void processGNSSInput() {
    LOG_DEBUG("Entering processGNSSInput");
    unsigned long pgiStart = millis();
    if (xSemaphoreTake(gnssSpiMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        LOG_ERROR("Failed to get SPI mutex in processGNSSInput");
        return;
    }
    
    unsigned long tStart = millis();
    GNSSSPI.beginTransaction(SPISettings(UBX_SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(UBX_CS, LOW);
    delayMicroseconds(1);

    uint8_t b;
    int consecutiveFFs = 0;
    int totalBytes = 0;

    while (consecutiveFFs < 10 && totalBytes < 512) {  // hard ceiling to prevent infinite loop
        b = GNSSSPI.transfer(0xFF);
        totalBytes++;

        if (b == 0xFF) {
            consecutiveFFs++;
        } else {
            consecutiveFFs = 0;
        }

        ubx_parse_byte(b);
    }


    digitalWrite(UBX_CS, HIGH);
    GNSSSPI.endTransaction();
    
    xSemaphoreGive(gnssSpiMutex);
    
    unsigned long tElapsed = millis() - tStart;
    if (tElapsed > 0) {
        LOG_NAV("GNSS checkUblox time, %lu", tElapsed);
    }
    LOG_NAV("processGNSSInputTime, %lu", millis() - pgiStart);
}

bool processRTKConnection() {
    LOG_DEBUG("processRTKConnection: start");
    unsigned long PRCstart = millis();
    bool pushedRTCM = false;
    uint8_t rtcmBuf[256]; // adjust if needed
    size_t byteCount = 0;

    unsigned long now = millis();  // capture current time once
    correctionAge = now - lastReceivedRTCM_ms;

    // Step 1: Read RTCM from NTRIP into local buffer
    if (xSemaphoreTake(ntripClientMutex, pdMS_TO_TICKS(mutexWait)) != pdTRUE) {
        LOG_ERROR("processRTKConnection ntripClientMutex timeout");
        return false;
    }

    if (ntripClient.connected() && ntripClient.available()) {
        while (ntripClient.available() && byteCount < sizeof(rtcmBuf)) {
            rtcmBuf[byteCount++] = ntripClient.read();
        }
    }
    xSemaphoreGive(ntripClientMutex);

    if (byteCount == 0) {
        LOG_DEBUG("processRTKConnection no RTCM bytes to push");
        return false;
    }
    
    // Step 2: Push buffered data to GNSS
    if (xSemaphoreTake(gnssSpiMutex, pdMS_TO_TICKS(mutexWait)) != pdTRUE) {
        LOG_ERROR("processRTKConnection gnssSpiMutex timeout");
        return false;
    }

    unsigned long PRDStart = millis();
    GNSSSPI.beginTransaction(SPISettings(UBX_SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(UBX_CS, LOW);
    delayMicroseconds(1);

    for (size_t i = 0; i < byteCount; i++) {
        GNSSSPI.transfer(rtcmBuf[i]);
    }

    digitalWrite(UBX_CS, HIGH);
    GNSSSPI.endTransaction();
    unsigned long PRDEnd = millis();
    xSemaphoreGive(gnssSpiMutex);
    LOG_NAV("pushRawData time, %lu", PRDEnd - PRDStart);
    lastReceivedRTCM_ms = millis();
    pushedRTCM = true;

    LOG_NAV("processRTKConnection pushed %u bytes in %lu ms",
              byteCount, PRDEnd - PRDStart);
    
    LOG_NAV("processRTKConnection time, %lu", PRDEnd - PRCstart);
    return pushedRTCM;
}

bool connectToNTRIP() {
    LOG_DEBUG("connectToNTRIP: starting");
    unsigned long ctnStart = millis();
    // Step 1: Try to acquire the mutex
    if (xSemaphoreTake(ntripClientMutex, pdMS_TO_TICKS(mutexWait)) != pdTRUE) {
        LOG_ERROR("connectToNTRIP ntripClientMutex timeout");
        return false;
    }

    // Step 2: Early out if already connected
    if (ntripClient.connected()) {
        xSemaphoreGive(ntripClientMutex);
        LOG_DEBUG("connectToNTRIP: already connected");
        return true;
    }

    // Step 3: Attempt socket connection
    if (!ntripClient.connect(casterHost, casterPort)) {
        LOG_ERROR("connectToNTRIP: failed to connect to %s:%d", casterHost, casterPort);
        xSemaphoreGive(ntripClientMutex);
        return false;
    }

    // Step 4: Format and send HTTP request
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
    LOG_DEBUG("connectToNTRIP: HTTP GET sent to %s:%u", casterHost, casterPort);

    // Step 5: Immediately release mutex so other tasks can run
    xSemaphoreGive(ntripClientMutex);

    // Step 6: Wait briefly for a response (non-blocking for others)
    vTaskDelay(pdMS_TO_TICKS(300)); // tuneable

    // Step 7: Reacquire the mutex to inspect the response
    if (xSemaphoreTake(ntripClientMutex, pdMS_TO_TICKS(mutexWait)) != pdTRUE) {
        LOG_ERROR("connectToNTRIP: couldn't reacquire mutex to read response");
        return false;
    }

    char response[256] = {0};
    int idx = 0;
    while (ntripClient.available() && idx < sizeof(response) - 1) {
        response[idx++] = ntripClient.read();
    }
    response[idx] = '\0';

    // Optional: check for success keyword in response
    if (strstr(response, "200 OK") != NULL || strstr(response, "ICY 200 OK") != NULL) {
        LOG_DEBUG("connectToNTRIP: server responded OK");
    } else {
        LOG_ERROR("connectToNTRIP: server response:\n%s", response);
    }

    bool finalStatus = ntripClient.connected();
    LOG_DEBUG("connectToNTRIP: socket state after handshake: connected=%d available=%d",
              finalStatus, ntripClient.available());

    xSemaphoreGive(ntripClientMutex);
    LOG_NAV("connectToNTRIP time, %lu", millis() - ctnStart);
    return finalStatus;
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
        const unsigned long start = millis();
        const unsigned long maxCorrectionAgeBeforeReconnect = 1000; 
        
        processGNSSInput();

        LOG_DEBUG("GNSSTask triggering NTRIP reconnect (age = %lu)", correctionAge);
        bool connected = connectToNTRIP();

        bool gotRTCM = processRTKConnection();
        if (!gotRTCM) {
            LOG_ERROR("GNSSTask processRTKConnection false");
        }

        LOG_DEBUG("GNSSTask stack high water mark: %u", uxTaskGetStackHighWaterMark(NULL));
        LOG_NAV("GNSSTask time, %lu", millis() - start);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    
}

// --- GGA generation ---
bool generateGGA(const UBX_NAV_PVT_data_t* pvt, char* out, size_t outLen) {
    if (!pvt || !out || outLen < 100) return false;
    unsigned long gggaStart = millis();
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
    
    LOG_NAV("generateGGA time, %lu", millis() - gggaStart);

    return true;
}

void GGATask(void *pvParameters) {
    LOG_DEBUG("GGATask Start");
    const TickType_t interval = pdMS_TO_TICKS(500); // 2 Hz
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true) {
        UBX_NAV_PVT_data_t currentPVT;
        const unsigned long ggaTstart = millis();
        if (xSemaphoreTake(gnssMutex, pdMS_TO_TICKS(mutexWait)) == pdTRUE) {
            memcpy(&currentPVT, (const void*)&lastValidPVT, sizeof(UBX_NAV_PVT_data_t));
            xSemaphoreGive(gnssMutex);
        } else {
            LOG_ERROR("GGATask gnssMutex timeout");
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
            if (xSemaphoreTake(ntripClientMutex, pdMS_TO_TICKS(mutexWait)) == pdTRUE) {
                if (ntripClient.connected()) {
                    ntripClient.print(gga);
                    LOG_NAV("GGA forwarded to caster: %s", gga);
                }
                xSemaphoreGive(ntripClientMutex);
            } else {
                LOG_ERROR("GGATask ntripClientMutex timeout");
            }
        }
        LOG_NAV("GGATask time, %lu", millis() - ggaTstart);
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