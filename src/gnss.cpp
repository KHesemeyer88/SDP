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

// GNSS object (declared in config.h)
SFE_UBLOX_GNSS myGPS;

// GNSS data structure
volatile GNSSData gnssData = {0};

// RTK Correction status tracking
volatile CorrectionStatus rtcmCorrectionStatus = CORR_NONE;
unsigned long correctionAge = 0;
unsigned long lastReceivedRTCM_ms = 0;
const unsigned long maxTimeBeforeHangup_ms = 10000UL; // 10 seconds timeout

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
            LOG_DEBUG("------------------------------------------");
            LOG_DEBUG("System time set from GNSS: %04d-%02d-%02d %02d:%02d (UTC)",
                pvtData->year, pvtData->month, pvtData->day, pvtData->hour, pvtData->min);
            LOG_DEBUG("------------------------------------------");
        }
        
        // Set flag to indicate new data is available
        gnssData.newDataAvailable = true;
        
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
    Wire.begin(32, 33); // Using default I2C pins
    
    if (!myGPS.begin()) {
        LOG_ERROR("u-blox GNSS not detected. Check wiring.");
        return false;
    }
    
    LOG_DEBUG("GNSS module found!");
    
    myGPS.setNavigationFrequency(NAV_FREQ);
    myGPS.setI2CInput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3);
    myGPS.setDGNSSConfiguration(SFE_UBLOX_DGNSS_MODE_FIXED);
    myGPS.setMainTalkerID(SFE_UBLOX_MAIN_TALKER_ID_GP);
    myGPS.setNMEAGPGGAcallbackPtr(&pushGPGGA);
    myGPS.setVal8(UBLOX_CFG_MSGOUT_NMEA_ID_GGA_I2C, 20);
    
    // Enable the callback for PVT messages
    myGPS.setAutoPVTcallbackPtr(&pvtCallback);
    
    // Disable sensor fusion
    if (!myGPS.setVal8(UBLOX_CFG_SFCORE_USE_SF, 0)) {
        LOG_ERROR("Failed to disable sensor fusion.");
    } else {
        LOG_DEBUG("IMU sensor fusion disabled.");
    }
    
    return true;
}
// Get fusion status from IMU
// Change the return type and implementation:
char* getFusionStatus(char* buffer, size_t bufferSize) {
    // Temporarily set nav frequency to 1Hz for status check
    myGPS.setNavigationFrequency(1);
    const char* defaultStatus = "Failed to get fusion data";
    
    // Initialize buffer with default message
    snprintf(buffer, bufferSize, "%s", defaultStatus);

    // Try to get ESF info with timeout
    unsigned long startTime = millis();
    const unsigned long ESF_TIMEOUT = 1000; // 1 second timeout

    if (!myGPS.getEsfInfo()) {
        while (millis() - startTime < ESF_TIMEOUT) {
            if (myGPS.getEsfInfo()) {
                uint8_t fusionMode = myGPS.packetUBXESFSTATUS->data.fusionMode;
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
// Process RTK connection
bool processGNSSConnection() {
    bool clientConnected = false;
    unsigned long startTime = millis();
    
    LOG_DEBUG("Attempting to take ntripClient mutex");
    
    // Take mutex before checking connection status
    if (xSemaphoreTake(ntripClientMutex, portMAX_DELAY) == pdTRUE) {
        LOG_DEBUG("ntripClient mutex acquired (after %lu ms)", millis() - startTime);
        clientConnected = ntripClient.connected();
        LOG_DEBUG("NTRIP client connected: %d", clientConnected);
        
        if (clientConnected && ntripClient.available()) {
            // Create a buffer large enough to hold multiple RTCM messages
            uint8_t rtcmBuffer[512*4];
            size_t rtcmCount = 0;
            
            unsigned long readStartTime = millis();
            // Collect all available RTCM data
            while (ntripClient.available() && rtcmCount < sizeof(rtcmBuffer)) {
                rtcmBuffer[rtcmCount++] = ntripClient.read();
            }
            
            LOG_DEBUG("Received %d bytes of RTCM data (read took %lu ms)", rtcmCount, millis() - readStartTime);
            
            // Release mutex before lengthy processing
            unsigned long mutexHeldTime = millis() - startTime;
            xSemaphoreGive(ntripClientMutex);
            LOG_DEBUG("Released ntripClient mutex after reading data (held for %lu ms)", mutexHeldTime);
            
            if (rtcmCount > 0) {
                // Update the timestamp for when we last received any correction data
                lastReceivedRTCM_ms = millis();
                
                unsigned long pushStartTime = millis();
                // Push all collected data to the GPS module at once
                myGPS.pushRawData(rtcmBuffer, rtcmCount);
                LOG_DEBUG("Pushed RTCM data to GPS module (took %lu ms)", millis() - pushStartTime);
            }
        } else {
            // Release mutex if not connected or no data available
            unsigned long mutexHeldTime = millis() - startTime;
            xSemaphoreGive(ntripClientMutex);
            LOG_DEBUG("Released ntripClient mutex (no data) (held for %lu ms)", mutexHeldTime);
        }
    } else {
        LOG_DEBUG("Failed to acquire ntripClient mutex");
    }
    
    // Check for timeout, but don't block - just report status
    if ((millis() - lastReceivedRTCM_ms) > maxTimeBeforeHangup_ms) {
        correctionAge = millis() - lastReceivedRTCM_ms;
        LOG_DEBUG("RTCM corrections timed out (%lu ms)", correctionAge);
        return false;
    }

    // Update correction status based on age
    correctionAge = millis() - lastReceivedRTCM_ms;
    LOG_DEBUG("RTCM correction age: %lu ms", correctionAge);
    
    CorrectionStatus oldStatus = rtcmCorrectionStatus;
    
    if (correctionAge < 5000) { // Less than 5 seconds old
        rtcmCorrectionStatus = CORR_FRESH;
    } else if (correctionAge < 30000) { // Less than 30 seconds old
        rtcmCorrectionStatus = CORR_STALE;
    } else {
        rtcmCorrectionStatus = CORR_NONE;
    }
    
    if (oldStatus != rtcmCorrectionStatus) {
        LOG_DEBUG("RTCM correction status changed: %d -> %d", oldStatus, rtcmCorrectionStatus);
    }
    
    LOG_PERF("Total GNSS connection processing took %lu ms", millis() - startTime);
    return clientConnected;
}

// Add this function to gnss.cpp or in a utility header file

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
    bool isConnected = false;

    // Take mutex before checking connection status
    if (xSemaphoreTake(ntripClientMutex, portMAX_DELAY) == pdTRUE) {
        isConnected = ntripClient.connected();
        
        if (!isConnected) {
            LOG_DEBUG("Connecting to NTRIP caster: %s", casterHost);
            
            if (!ntripClient.connect(casterHost, casterPort)) {
                LOG_ERROR("Connection to caster failed");
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
                    LOG_ERROR("Caster timed out!");
                    ntripClient.stop();
                    xSemaphoreGive(ntripClientMutex);
                    return false;
                }
                
                // Temporarily release mutex during delay to prevent blocking other tasks
                xSemaphoreGive(ntripClientMutex);
                vTaskDelay(pdMS_TO_TICKS(10));
                if (xSemaphoreTake(ntripClientMutex, portMAX_DELAY) != pdTRUE) {
                    // If we can't get the mutex back, something is wrong
                    return false;
                }
                
                // Recheck if connection is still valid
                if (!ntripClient.connected()) {
                    LOG_ERROR("Connection lost during response wait");
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
                        LOG_ERROR("Bad credentials! Check caster username and password.");
                        connectionResult = 401;
                    }
                }
            }
            
            response[responseSpot] = '\0'; // NULL-terminate the response
            
            if (connectionResult != 200) {
                LOG_ERROR("Failed to connect to %s", casterHost);
                xSemaphoreGive(ntripClientMutex);
                return false;
            } else {
                LOG_DEBUG("Connected to: %s", casterHost);
                lastReceivedRTCM_ms = millis(); // Reset timeout
                isConnected = true;
            }
        }
        
        // Release mutex
        xSemaphoreGive(ntripClientMutex);
    }
    
    return isConnected;
}

// GNSS task function
void GNSSTask(void *pvParameters) {
    LOG_DEBUG("GNSS Task Started");
    
    // Initialize GNSS module
    if (!initializeGNSS()) {
        LOG_ERROR("GNSS initialization failed. Retrying...");
        
        int retryCount = 0;
        while (!initializeGNSS()) {
            retryCount++;
            LOG_ERROR("GNSS initialization retry #%d", retryCount);
            vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second between retries
            
            // After 10 retries, reset ESP32
            if (retryCount >= 10) {
                LOG_ERROR("GNSS initialization failed after 10 attempts. Resetting...");
                ESP.restart();
            }
        }
    }
    
    // Main task loop
    unsigned long lastRTKConnectionAttempt = 0;
    
    for (;;) {
        unsigned long loopStartTime = millis();
        
        // Poll the GNSS module
        unsigned long pollStartTime = millis();
        myGPS.checkUblox();
        myGPS.checkCallbacks();
        unsigned long pollTime = millis() - pollStartTime;
        if (pollTime > 100) {  // Only log if it took significant time
            LOG_PERF("GNSS polling took %lu ms", pollTime);
        }
        
        // Process RTK connection with timing
        unsigned long rtcmStartTime = millis();
        bool rtcmConnected = processGNSSConnection();
        unsigned long rtcmProcessTime = millis() - rtcmStartTime;
        if (rtcmProcessTime > 100) {  // Only log if it took significant time
            LOG_PERF("RTCM connection processing took %lu ms", rtcmProcessTime);
        }
        
        // Attempt to reconnect to NTRIP if needed
        if (!rtcmConnected) {
            unsigned long currentTime = millis();
            if (currentTime - lastRTKConnectionAttempt > 5000) { // Try every 5 seconds
                lastRTKConnectionAttempt = currentTime;
                
                unsigned long connectStartTime = millis();
                bool connected = connectToNTRIP();
                LOG_DEBUG("NTRIP connection attempt %s (took %lu ms)", 
                          connected ? "successful" : "failed", 
                          millis() - connectStartTime);
            }
        }
        
        // Send GNSS data to WebSocket clients if new data is available
        unsigned long dataStartTime = millis();
        if (xSemaphoreTake(gnssMutex, pdMS_TO_TICKS(100)) == pdTRUE) {  // 100ms timeout
            LOG_DEBUG("GNSS task acquired GNSS mutex (after %lu ms)", millis() - dataStartTime);
            
            if (gnssData.newDataAvailable) {
                // Let WebSocket task handle data sending to avoid potential issues
                LOG_DEBUG("New GNSS data available");
                gnssData.newDataAvailable = false;
            }
            
            unsigned long mutexHeldTime = millis() - dataStartTime;
            xSemaphoreGive(gnssMutex);
            LOG_DEBUG("GNSS task released GNSS mutex (held for %lu ms)", mutexHeldTime);
        } else {
            LOG_DEBUG("GNSS task failed to acquire GNSS mutex within timeout");
        }
        
        // Log total loop time if significant
        unsigned long loopTime = millis() - loopStartTime;
        if (loopTime > 200) {  // Only log if the loop took a significant amount of time
            LOG_PERF("GNSS task loop iteration took %lu ms", loopTime);
        }
        
        // Use a short delay to prevent task starvation
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz update rate
    }
}