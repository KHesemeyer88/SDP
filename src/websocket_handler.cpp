#include "websocket_handler.h"
#include "rtos_tasks.h"
#include "gnss.h"
#include <WebSocketsServer.h>
#include "logging.h"

// WebSocket server instance
WebSocketsServer webSocket(81);

// Timing variables for updates
unsigned long lastWSSensorUpdate = 0;
unsigned long lastWSGPSUpdate = 0;
unsigned long lastWSRTKUpdate = 0;
unsigned long lastWSStatsUpdate = 0;

// WebSocket event handler for RTOS
void webSocketEventRTOS(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            LOG_ERROR("WebSocket client #%u disconnected", num);
            break;
            
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket.remoteIP(num);
                LOG_DEBUG("WebSocket client #%u connected from %d.%d.%d.%d", 
                    num, ip[0], ip[1], ip[2], ip[3]);
                
                // Send initial state to newly connected client
                // (To be implemented later when we add sensor/GPS functionality)
            }
            break;
            
        case WStype_TEXT:
            {
                LOG_DEBUG("WebSocket received text from client #%u: length=%d", num, length);
                // Parse the JSON command
                DynamicJsonDocument doc(512);
                DeserializationError error = deserializeJson(doc, payload);
                
                if (error) {
                    LOG_ERROR("JSON parsing failed: %s", error.c_str());
                    return;
                }
                
                // Fast path for control messages (manual driving)
                if (doc.containsKey("control")) {
                    LOG_DEBUG("Received control command: v=%f, h=%f", 
                        (float)doc["control"]["vertical"], 
                        (float)doc["control"]["horizontal"]);
                    // Create a command struct to send to the control task
                    ControlCommand cmd;
                    cmd.type = CMD_MANUAL_CONTROL;
                    
                    // Get joystick values
                    float normalizedY = doc["control"]["vertical"];
                    float normalizedX = doc["control"]["horizontal"];
                    
                    // Set driving state for non-neutral commands
                    const float JOYSTICK_DEADZONE = 0.03f;
                    if (abs(normalizedY) > JOYSTICK_DEADZONE || abs(normalizedX) > JOYSTICK_DEADZONE) {
                        cmd.manual.isDriving = true;
                    } else {
                        cmd.manual.isDriving = false;
                    }
                    
                    cmd.manual.throttle = normalizedY;
                    cmd.manual.steering = normalizedX;
                    
                    // Send to command queue with timeout
                    if (commandQueue != NULL) {
                        if (xQueueSendToBack(commandQueue, &cmd, 0) != pdPASS) {
                        }
                    }
                    
                    // Command processed, return immediately
                    return;
                }
                
                // Other command types will be implemented later
                // For now, we're just focusing on manual driving
            }
            break;

            case WStype_ERROR:
                LOG_ERROR("WebSocket error for client #%u", num);
                break;
            
            case WStype_BIN:
                LOG_DEBUG("WebSocket received binary data from client #%u: length=%d", num, length);
                break;
        }
}

// Send a status message
void sendStatusMessage(const String& message) {
    DynamicJsonDocument doc(128);
    doc["type"] = "status";
    doc["message"] = message;
    
    String jsonString;
    serializeJson(doc, jsonString);
    webSocket.broadcastTXT(jsonString);
}

// Send an error message
void sendErrorMessage(const String& message) {
    DynamicJsonDocument doc(128);
    doc["type"] = "error";
    doc["message"] = message;
    
    String jsonString;
    serializeJson(doc, jsonString);
    webSocket.broadcastTXT(jsonString);
}

// WebSocket task implementation
void WebSocketTask(void *pvParameters) {
    // Initialize WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEventRTOS);
    LOG_DEBUG("WebSocket server started on port 81");
    
    unsigned long lastStatusLog = 0;
    unsigned long lastSystemStatsLog = 0;
    unsigned long currentTime = 0;
    
    // Task loop
    for (;;) {
        unsigned long loopStartTime = millis();
        
        // Handle WebSocket events
        unsigned long wsLoopStartTime = millis();
        webSocket.loop();
        unsigned long wsLoopTime = millis() - wsLoopStartTime;
        if (wsLoopTime > 50) {  // Only log if it took significant time
            LOG_DEBUG("WebSocket.loop() took %lu ms", wsLoopTime);
        }
        
        // Update current time once per loop
        currentTime = millis();
        
        // Log connection status periodically (every 5 seconds)
        if (currentTime - lastStatusLog >= 5000) {
            lastStatusLog = currentTime;
            LOG_DEBUG("WebSocket clients connected: %d", webSocket.connectedClients());
        }
        
        // Log system stats periodically (every 30 seconds)
        if (currentTime - lastSystemStatsLog >= 30000) {
            lastSystemStatsLog = currentTime;
            LOG_DEBUG("System stats: Free heap: %u bytes", ESP.getFreeHeap());
            LOG_DEBUG("WebSocket task stack high water mark: %u", uxTaskGetStackHighWaterMark(websocketTaskHandle));
            LOG_DEBUG("GNSS task stack high water mark: %u", uxTaskGetStackHighWaterMark(gnssTaskHandle));
            LOG_DEBUG("Control task stack high water mark: %u", uxTaskGetStackHighWaterMark(controlTaskHandle));
        }
        
        // Send periodic updates with timing information
        if (currentTime - lastWSSensorUpdate >= WS_SENSOR_UPDATE_INTERVAL) {
            unsigned long updateStartTime = millis();
            sendSensorData();
            unsigned long updateTime = millis() - updateStartTime;
            if (updateTime > 20) {  // Only log if it took significant time
                LOG_DEBUG("Sensor data update took %lu ms", updateTime);
            }
            lastWSSensorUpdate = currentTime;
        }
        
        if (currentTime - lastWSGPSUpdate >= WS_GPS_UPDATE_INTERVAL) {
            unsigned long gpsStartTime = millis();
            LOG_DEBUG("Sending GPS data update");
            sendGPSData();
            unsigned long gpsUpdateTime = millis() - gpsStartTime;
            if (gpsUpdateTime > 20) {  // Only log if it took significant time
                LOG_DEBUG("GPS data update took %lu ms", gpsUpdateTime);
            }
            lastWSGPSUpdate = currentTime;
        }
        
        if (currentTime - lastWSRTKUpdate >= WS_RTK_UPDATE_INTERVAL) {
            unsigned long rtkStartTime = millis();
            LOG_DEBUG("Sending RTK status update");
            sendRTKStatus();
            unsigned long rtkUpdateTime = millis() - rtkStartTime;
            if (rtkUpdateTime > 20) {  // Only log if it took significant time
                LOG_DEBUG("RTK status update took %lu ms", rtkUpdateTime);
            }
            lastWSRTKUpdate = currentTime;
        }
        
        // Calculate and log the total loop time if significant
        unsigned long loopTime = millis() - loopStartTime;
        if (loopTime > 50) {  // Only log if the loop took a significant amount of time
            LOG_DEBUG("WebSocket task loop iteration took %lu ms", loopTime);
        }
        
        // Small yield to allow other tasks to run
        vTaskDelay(1);
    }
}

// These function implementations will be added later when implementing
// sensor reading and GPS functionality

void sendSensorData(uint8_t clientNum) {
    // To be implemented
}

// Send GPSData implementation
void sendGPSData(uint8_t clientNum) {
    unsigned long startTime = millis(); // Add this new variable
    // Log mutex acquisition attempts
    LOG_DEBUG("Attempting to take GNSS mutex for GPS data update");
    // Create JSON document for GPS data
    DynamicJsonDocument doc(256);
    doc["type"] = "gps";
    
    // Take mutex to safely access GNSS data
    if (xSemaphoreTake(gnssMutex, portMAX_DELAY) == pdTRUE) {
        unsigned long mutexAcquiredTime = millis();
        LOG_DEBUG("WebSocket: GNSS mutex acquired after %lu ms", mutexAcquiredTime - startTime);
        // Add GPS information
        if (gnssData.fixType > 0) {
            LOG_DEBUG("GPS data: fix=%d, lat=%.7f, lng=%.7f", 
                gnssData.fixType, gnssData.latitude, gnssData.longitude);
            // Add fix type description
            String fixDesc;
            switch (gnssData.fixType) {
                case 1: fixDesc = "1 (Dead Reckoning)"; break;
                case 2: fixDesc = "2 (2D)"; break;
                case 3: fixDesc = "3 (3D)"; break;
                case 4: fixDesc = "4 (GNSS+DR)"; break;
                case 5: fixDesc = "5 (Time Only)"; break;
                default: fixDesc = "Unknown"; break;
            }
            doc["fix"] = fixDesc;
            
            // Add coordinates with precision based on fix type
            if (gnssData.fixType >= 2) {
                doc["lat"] = String(gnssData.latitude, 7); // 7 decimal places for high precision
                doc["lng"] = String(gnssData.longitude, 7);
            }
        } else {
            LOG_DEBUG("GPS data: No fix available");
            doc["fix"] = "No Fix";
        }
        
        // Release mutex
        unsigned long beforeReleaseTime = millis();
        xSemaphoreGive(gnssMutex);
        LOG_DEBUG("GNSS mutex released after GPS data update (held for %lu ms)", millis() - beforeReleaseTime);
    } else {
        LOG_ERROR("Failed to acquire GNSS mutex for GPS data update");
        // If we couldn't get the mutex, send minimal data
        doc["fix"] = "Data Unavailable";
    }
    
    // Serialize and send JSON
    String jsonString;
    serializeJson(doc, jsonString);
    
    if (clientNum == 255) {
        LOG_DEBUG("Broadcasting GPS data to all clients: %s", jsonString.c_str());
        // Broadcast to all clients
        webSocket.broadcastTXT(jsonString);
    } else {
        LOG_DEBUG("Sending GPS data to client #%u: %s", clientNum, jsonString.c_str());
        // Send to specific client
        webSocket.sendTXT(clientNum, jsonString);
    }
    LOG_DEBUG("Total GPS data update process took %lu ms", millis() - startTime);
}

void sendRTKStatus(uint8_t clientNum) {
    // Create JSON document for RTK data
    DynamicJsonDocument doc(256);
    doc["type"] = "rtk";
    
    // Take mutex to safely access GNSS data
    if (xSemaphoreTake(gnssMutex, portMAX_DELAY) == pdTRUE) {
        // Add RTK correction status
        switch (rtcmCorrectionStatus) {
            case CORR_FRESH: doc["status"] = "Fresh"; break;
            case CORR_STALE: doc["status"] = "Stale"; break;
            case CORR_NONE: doc["status"] = "None"; break;
        }
        
        // Add correction age
        doc["age"] = correctionAge;
        
        // Take mutex for ntripClient access
        bool connectionStatus = false;
        if (xSemaphoreTake(ntripClientMutex, portMAX_DELAY) == pdTRUE) {
            // Add RTK connection status
            connectionStatus = ntripClient.connected();
            xSemaphoreGive(ntripClientMutex);
        }
        doc["connected"] = connectionStatus;
        
        // Add RTK solution status (carrier solution)
        doc["carrSoln"] = gnssData.carrSoln;
        
        // Add horizontal accuracy
        doc["hAcc"] = gnssData.hAcc;
        
        // Add fix type
        doc["fixType"] = gnssData.fixType;
        
        // Release GNSS mutex
        xSemaphoreGive(gnssMutex);
    } else {
        // If we couldn't get the mutex, send minimal data
        doc["status"] = "Data Unavailable";
    }
    
    // Serialize and send JSON
    String jsonString;
    serializeJson(doc, jsonString);
    
    if (clientNum == 255) {
        // Broadcast to all clients
        webSocket.broadcastTXT(jsonString);
    } else {
        // Send to specific client
        webSocket.sendTXT(clientNum, jsonString);
    }
}

void sendNavigationStats(uint8_t clientNum) {
    // To be implemented
}