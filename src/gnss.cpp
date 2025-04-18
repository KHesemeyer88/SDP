#include "gnss.h"
#include "websocket_handler.h"
#include "logging.h"
#include <sys/time.h>

// Flag to track if the system time has been set
static bool systemTimeSet = false;

// GNSS task handle
TaskHandle_t gnssTaskHandle = NULL;

// Mutex for accessing GNSS data
SemaphoreHandle_t gnssMutex = NULL;

// GNSS object (declared in gnss.h)
SFE_UBLOX_GNSS_SPI myGPS;

// GNSS data structure
volatile GNSSData gnssData = {0};

// RTK Correction status tracking
volatile CorrectionStatus rtcmCorrectionStatus = CORR_NONE;
unsigned long correctionAge = 0;
unsigned long lastReceivedRTCM_ms = 0;
const unsigned long maxTimeBeforeHangup_ms = 10000UL; // 10 seconds timeout

static int mutexWait = 50;

// PVT callback function - updates the gnssData structure with new data
void pvtCallback(UBX_NAV_PVT_data_t *pvtData) {
    // Take mutex to safely update the shared data
    if (xSemaphoreTake(gnssMutex, 0) == pdTRUE) {
        // Update global position variables with new data
        gnssData.latitude = pvtData->lat / 10000000.0;
        gnssData.longitude = pvtData->lon / 10000000.0;
        gnssData.speed = pvtData->gSpeed / 1000.0;  // Convert from mm/s to m/s
        gnssData.fixType = pvtData->fixType;
        gnssData.heading = pvtData->headMot / 100000.0;
        gnssData.carrSoln = pvtData->flags.bits.carrSoln;
        gnssData.hAcc = pvtData->hAcc / 10.0; // cm

        // Set system time once, after acquiring a valid 3D fix
        if (!systemTimeSet && pvtData->fixType >= 3) {  
            struct tm timeinfo = {0};
            timeinfo.tm_year = pvtData->year - 1900;
            timeinfo.tm_mon  = pvtData->month - 1;
            timeinfo.tm_mday = pvtData->day;
            timeinfo.tm_hour = pvtData->hour;
            timeinfo.tm_min  = pvtData->min;
            timeinfo.tm_sec  = 0; // Set seconds to 0 since you don't need precision

            time_t epochTime = mktime(&timeinfo);
            struct timeval tv = { .tv_sec = epochTime, .tv_usec = 0 };
            settimeofday(&tv, NULL);

            systemTimeSet = true;  // Prevent future updates
            LOG_ERROR("------------------------------------------");
            LOG_ERROR("GNSS SYS TIME: %04d-%02d-%02d %02d:%02d (UTC)",
                pvtData->year, pvtData->month, pvtData->day, pvtData->hour, pvtData->min);
            LOG_ERROR("------------------------------------------");
        }
        
        // Set flag to indicate new data is available
        gnssData.newDataAvailable = true;
        gnssData.gnssFixTime = millis();
        
        xSemaphoreGive(gnssMutex);
    }
    // If we couldn't get the mutex, we'll skip this update
}

// NMEA GGA callback function - provides data to NTRIP caster
void pushGPGGA(NMEA_GGA_data_t *nmeaData) {
    // Take mutex before checking and using ntripClient
    if (xSemaphoreTake(ntripClientMutex, 0) == pdTRUE) {  // Use zero timeout to not block callback
        if (ntripClient.connected()) {
            ntripClient.print((const char *)nmeaData->nmea);
        }
        xSemaphoreGive(ntripClientMutex);
    }
}

// Initialize GNSS module
bool initializeGNSS() {
    LOG_DEBUG("STARTING initializeGNSS");
    
    // Configure CS, INT and RST pins
    pinMode(NAV_CS_PIN, OUTPUT);
    digitalWrite(NAV_CS_PIN, HIGH);  // Default CS to high/inactive
    pinMode(NAV_INT_PIN, INPUT);
    pinMode(NAV_RST_PIN, OUTPUT);
    digitalWrite(NAV_RST_PIN, HIGH); // Default to not resetting
    
    // Initialize the SPI bus with your defined pins
    static SPIClass GNSSSPI(HSPI);
    GNSSSPI.begin(NAV_SCK_PIN, NAV_MISO_PIN, NAV_MOSI_PIN, NAV_CS_PIN);

    delay(100);

    // Dummy SPI sync
    GNSSSPI.beginTransaction(SPISettings(NAV_SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
    GNSSSPI.transfer(0xFF);
    GNSSSPI.endTransaction();
    delay(100);
    
    // Use SPI to begin communication with GPS module - note reduced frequency for stability
    if (!myGPS.begin(GNSSSPI, NAV_CS_PIN, NAV_SPI_FREQUENCY)) {  
        LOG_ERROR("u-blox GNSS not detected over SPI. Check wiring.");
        return false;
    }
    
    LOG_DEBUG("GNSS module found over SPI!");
    
    // All configuration first, while UBX input is still accepted
    if (!myGPS.setNavigationFrequency(NAV_FREQ)) {
        LOG_ERROR("Failed to set NAV FREQ, %d", NAV_FREQ);
    } else {
        LOG_DEBUG("set NAV FREQ, %d", NAV_FREQ);
    }

    if (!myGPS.setVal8(UBLOX_CFG_NAVSPG_DYNMODEL, 3)) {
        LOG_ERROR("Failed to set dynamic model");
    } else {
        LOG_DEBUG("set DYNMODEL, %d", 3);
    }

    if (!myGPS.setVal8(UBLOX_CFG_SFCORE_USE_SF, 0)) {
        LOG_ERROR("Failed to disable sensor fusion");
    } else {
        LOG_DEBUG("Sensor fusion disabled");
    }

    if (!myGPS.setVal8(UBLOX_CFG_MSGOUT_UBX_NAV_PVT_SPI, NAV_FREQ)) {
        LOG_ERROR("Failed to enable UBX-NAV-PVT output");
    } else {
        LOG_DEBUG("set UBX_NAV_PVT rate to NAV FREQ, %d", NAV_FREQ);
    }

    if (!myGPS.setVal8(UBLOX_CFG_MSGOUT_NMEA_ID_GGA_SPI, 1)) {  // reasonable rate
        LOG_ERROR("Failed to enable GGA output");
    } else {
        LOG_DEBUG("set GGA output rate to, %d", 1);
    }

    if (!myGPS.setNMEAGPGGAcallbackPtr(&pushGPGGA)) {
        LOG_ERROR("Failed to set GGA callback");
    } else {
        LOG_DEBUG("set GGA callback successfuly");
    }

    if (!myGPS.setAutoPVTcallbackPtr(&pvtCallback)) {
        LOG_ERROR("Failed to set PVT callback");
    } else {
        LOG_DEBUG("set PVT callback successfully");
    }

    uint8_t dynModel = 0;
    if (myGPS.getVal8(UBLOX_CFG_NAVSPG_DYNMODEL, &dynModel)) {
        LOG_DEBUG("Confirmed DYNMODEL = %d", dynModel);
    } else {
        LOG_ERROR("Failed to read back DYNMODEL");
    }

    uint8_t ggaRate = 0;
    if (myGPS.getVal8(UBLOX_CFG_MSGOUT_NMEA_ID_GGA_SPI, &ggaRate)) {
        LOG_DEBUG("Confirmed GGA rate = %d", ggaRate);
    } else {
        LOG_ERROR("Failed to read GGA output rate");
    }

    uint8_t pvtRate = 0;
    if (myGPS.getVal8(UBLOX_CFG_MSGOUT_UBX_NAV_PVT_SPI, &pvtRate)) {
        LOG_DEBUG("Confirmed NAV-PVT output rate = %d", pvtRate);
    } else {
        LOG_ERROR("Failed to read PVT output rate");
    }

    // Only now restrict input/output
    myGPS.setSPIInput(COM_TYPE_RTCM3);
    myGPS.setSPIOutput(COM_TYPE_UBX);
    
    return true;
}

// Get fusion status from IMU
// Change the return type and implementation:
char* getFusionStatus(char* buffer, size_t bufferSize) {
    LOG_DEBUG("getFusionStatus");
    // Temporarily set nav frequency to 1Hz for status check
    myGPS.setNavigationFrequency(1);
    const char* defaultStatus = "Failed to get fusion data";
    
    // Initialize buffer with default message
    snprintf(buffer, bufferSize, "%s", defaultStatus);

    // Try to get ESF info with timeout
    unsigned long startTime = millis();
    const unsigned long ESF_TIMEOUT = 2000; 
    if (!myGPS.getEsfInfo()) {
        while (millis() - startTime < ESF_TIMEOUT) {
            LOG_DEBUG("getEsfInfo");
            if (myGPS.getEsfInfo()) {
                uint8_t fusionMode = myGPS.packetUBXESFSTATUS->data.fusionMode;
                LOG_DEBUG("fusionMode, %s", fusionMode);
                switch(fusionMode) {
                    case 0: snprintf(buffer, bufferSize, "Initializing"); break;
                    case 1: snprintf(buffer, bufferSize, "Calibrated"); break;
                    case 2: snprintf(buffer, bufferSize, "Suspended"); break;
                    case 3: snprintf(buffer, bufferSize, "Disabled"); break;
                }
                break;
            }
        }
    }

    // Restore original navigation frequency
    myGPS.setNavigationFrequency(NAV_FREQ);
    return buffer;
}

// Process RTK connection
bool processRTKConnection() {
    //LOG_DEBUG("processGNSSConnection");
    bool clientConnected = false;
    bool dataAvailable = false;
    unsigned long startTime = millis();
    
    // Step 1: Quick mutex lock just to check status
    if (xSemaphoreTake(ntripClientMutex, pdMS_TO_TICKS(mutexWait)) == pdTRUE) {
        //LOG_DEBUG("ntrip mutex acq time, %lu", millis() - startTime);
        clientConnected = ntripClient.connected();
        //LOG_DEBUG("ntrip client conn., %d", clientConnected);
        
        // Check if data is available but don't read it yet
        dataAvailable = clientConnected && ntripClient.available();
        
        // If no data available, release mutex immediately
        if (!dataAvailable) {
            unsigned long mutexHeldTime = millis() - startTime;
            xSemaphoreGive(ntripClientMutex);
            //LOG_DEBUG("ntripClientMutex hold (no data) time, %lu", mutexHeldTime);
        }
    } else {
        LOG_ERROR("ntrip mutex fail");
        return false;
    }
    
    // Step 2: Read data only if available (still holding mutex if data is available)
    if (dataAvailable) {
        // Create a buffer large enough to hold multiple RTCM messages
        uint8_t rtcmBuffer[512*4];
        size_t rtcmCount = 0;
        
        unsigned long readStartTime = millis();
        // Collect all available RTCM data
        while (ntripClient.available() && rtcmCount < sizeof(rtcmBuffer)) {
            rtcmBuffer[rtcmCount++] = ntripClient.read();
        }
        
        //LOG_DEBUG("ntripClient.read time, %lu", millis() - readStartTime);
        //LOG_DEBUG("RTCM data bytes, %d", rtcmCount);
        
        // Release mutex before lengthy processing
        unsigned long mutexHeldTime = millis() - startTime;
        xSemaphoreGive(ntripClientMutex);
        //LOG_DEBUG("ntripClientMutex hold time, %lu", mutexHeldTime);
        
        if (rtcmCount > 0) {
            // Update the timestamp for when we last received any correction data
            lastReceivedRTCM_ms = millis();
            
            // Push all collected data to the GPS module at once
            myGPS.pushRawData(rtcmBuffer, rtcmCount);
            LOG_DEBUG("pushRawData time, %lu", millis() - lastReceivedRTCM_ms);
        }
    }
    
    // Check for timeout, but don't block - just report status
    if ((millis() - lastReceivedRTCM_ms) > maxTimeBeforeHangup_ms) {
        correctionAge = millis() - lastReceivedRTCM_ms;
        //LOG_DEBUG("RTCM timeout, %lu", correctionAge);
        return false;
    }

    // Update correction status based on age
    correctionAge = millis() - lastReceivedRTCM_ms;
    //LOG_DEBUG("RTCM age time, %lu", correctionAge);
    
    CorrectionStatus oldStatus = rtcmCorrectionStatus;
    
    if (correctionAge < 5000) { // Less than 5 seconds old
        rtcmCorrectionStatus = CORR_FRESH;
    } else if (correctionAge < 30000) { // Less than 30 seconds old
        rtcmCorrectionStatus = CORR_STALE;
    } else {
        rtcmCorrectionStatus = CORR_NONE;
    }
    
    if (oldStatus != rtcmCorrectionStatus) {
        //LOG_DEBUG("RTCM status chg., %d, %d", oldStatus, rtcmCorrectionStatus);
    }
    
    //LOG_PERF("processGNSSConnection time, %lu", millis() - startTime);
    return clientConnected;
}

// Simple base64 encoding function that works with fixed buffers
void base64Encode(const char* input, char* output, size_t outputSize) {
    static const char base64Chars[] = 
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    
    size_t inputLen = strlen(input);
    size_t i = 0;
    size_t j = 0;
    uint8_t char_array_3[3];
    uint8_t char_array_4[4];
    
    while (inputLen--) {
        char_array_3[i++] = *(input++);
        if (i == 3) {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;
            
            for (i = 0; i < 4; i++) {
                if (j < outputSize - 1) { // Leave room for null terminator
                    output[j++] = base64Chars[char_array_4[i]];
                }
            }
            i = 0;
        }
    }
    
    if (i) {
        for (size_t k = i; k < 3; k++) {
            char_array_3[k] = '\0';
        }
        
        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        
        for (size_t k = 0; k < i + 1; k++) {
            if (j < outputSize - 1) { // Leave room for null terminator
                output[j++] = base64Chars[char_array_4[k]];
            }
        }
        
        while (i++ < 3) {
            if (j < outputSize - 1) { // Leave room for null terminator
                output[j++] = '=';
            }
        }
    }
    
    output[j] = '\0'; // Null terminate the output string
}

// Connect to NTRIP caster
bool connectToNTRIP() {
    //LOG_DEBUG("connectTONTRIP");
    bool isConnected = false;

    // Take mutex before checking connection status
    if (xSemaphoreTake(ntripClientMutex, portMAX_DELAY) == pdTRUE) {
        isConnected = ntripClient.connected();
        
        if (!isConnected) {
            
            if (!ntripClient.connect(casterHost, casterPort)) {
                LOG_ERROR("!ntripClient.connect, %s, %s", casterHost, casterPort);
                xSemaphoreGive(ntripClientMutex);
                return false;
            }
        
            // Prepare auth string and encode it
            char authString[128]; // Buffer for username:password
            char encodedAuth[200]; // Buffer for base64 encoded auth string
            
            // Safely combine username and password
            snprintf(authString, sizeof(authString), "%s:%s", casterUser, casterUserPW);
            
            // Use our fixed-buffer base64 encoding function
            base64Encode(authString, encodedAuth, sizeof(encodedAuth));
            
            // Formulate the NTRIP request
            char serverRequest[512];
            snprintf(serverRequest, sizeof(serverRequest),
                   "GET /%s HTTP/1.0\r\n"
                   "User-Agent: NTRIP SparkFun u-blox Client v1.0\r\n"
                   "Accept: */*\r\n"
                   "Connection: close\r\n"
                   "Authorization: Basic %s\r\n"
                   "\r\n",
                   mountPoint, encodedAuth);
            
            ntripClient.write(serverRequest, strlen(serverRequest));
            
            // Wait for response with mutex held (non-blocking with vTaskDelay)
            unsigned long startTime = millis();
            while (ntripClient.available() == 0) {
                if (millis() > (startTime + 5000)) {
                    LOG_ERROR("caster timeout");
                    ntripClient.stop();
                    xSemaphoreGive(ntripClientMutex);
                    return false;
                }
                
                // Temporarily release mutex during delay to prevent blocking other tasks
                xSemaphoreGive(ntripClientMutex);
                vTaskDelay(pdMS_TO_TICKS(mutexWait));
                if (xSemaphoreTake(ntripClientMutex, portMAX_DELAY) != pdTRUE) {
                    // If we can't get the mutex back, something is wrong
                    return false;
                }
                
                // Recheck if connection is still valid
                if (!ntripClient.connected()) {
                    LOG_ERROR("!ntripClient.connected()");
                    xSemaphoreGive(ntripClientMutex);
                    return false;
                }
            }
            
            // Check reply
            int connectionResult = 0;
            char response[512];
            size_t responseSpot = 0;
            
            while (ntripClient.available()) { // Read bytes from the caster and store them
                if (responseSpot == sizeof(response) - 1) // Exit the loop if we get too much data
                    break;
                    
                response[responseSpot++] = ntripClient.read();
                
                if (connectionResult == 0) { // Only print success/fail once
                    if (strstr(response, "200") != nullptr) { //Look for '200 OK'
                        connectionResult = 200;
                    }
                    if (strstr(response, "401") != nullptr) { //Look for '401 Unauthorized'
                        LOG_ERROR("bad user/pw");
                        connectionResult = 401;
                    }
                }
            }
            
            response[responseSpot] = '\0'; // NULL-terminate the response
            
            if (connectionResult != 200) {
                LOG_ERROR("connectionResult != 200, %s", casterHost);
                xSemaphoreGive(ntripClientMutex);
                return false;
            } else {
                LOG_DEBUG("connectionResult = 200, %s", casterHost);
                lastReceivedRTCM_ms = millis(); // Reset timeout
                isConnected = true;
            }
        }
        
        // Release mutex
        xSemaphoreGive(ntripClientMutex);
    }
    //LOG_DEBUG("connectTONTRIP end");
    return isConnected;
}

// GNSS task function
void GNSSTask(void *pvParameters) {
    //LOG_DEBUG("GNSSTask");
    TickType_t xLastWakeTime;
    // Initialize time for consistent frequency
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / NAV_FREQ); // tie gnss task to nav board freq
    
    // Initialize GNSS module
    if (!initializeGNSS()) {
        LOG_ERROR("!initializeGNSS");
        
        int retryCount = 0;
        while (!initializeGNSS()) {
            retryCount++;
            LOG_ERROR("GNSS retry, %d", retryCount);
            vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second between retries
            
            // After 10 retries, reset ESP32
            if (retryCount >= 10) {
                handleSystemError("10 gnss fails, sys reset", true);
            }
        }
    }
    
    // Main task loop
    unsigned long lastRTKConnectionAttempt = 0;
    
    for (;;) {
        unsigned long loopStartTime = millis();
        
        // Poll the GNSS module
        myGPS.checkUblox();
        unsigned long checkUbloxTime = millis() - loopStartTime;
        if (checkUbloxTime > 0) {  // Only log if it took significant time
            LOG_DEBUG("GNSS checkUblox time, %lu", checkUbloxTime);
        }
        myGPS.checkCallbacks();
        unsigned long checkCallbacksTime = millis() - loopStartTime;
        if (checkCallbacksTime > 0) {  // Only log if it took significant time
            LOG_DEBUG("GNSS checkCallbacks time, %lu", checkCallbacksTime);
        }
        
        // Process RTK connection 
        bool rtcmConnected = processRTKConnection();
        
        // Attempt to reconnect to NTRIP if needed
        if (!rtcmConnected) {
            unsigned long currentTime = millis();
            if (currentTime - lastRTKConnectionAttempt > 5000) { // Try every 5 seconds
                lastRTKConnectionAttempt = currentTime;
                
                unsigned long connectStartTime = millis();
                bool connected = connectToNTRIP();
                //LOG_DEBUG("connectToNTRIP time, %lu", millis() - connectStartTime);
                //LOG_DEBUG("connectToNTRIP status, %s", connected ? "successful" : "failed");
            }
        }
        
        // Log total loop time if significant
        unsigned long loopTime = millis() - loopStartTime;
        if (loopTime > 0) {  // Only log if the loop took a significant amount of time
            LOG_DEBUG("GNSSTask time, %lu", loopTime);
        }
        
        // Tie task frequency to NAV_FREQ
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}