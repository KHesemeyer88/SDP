#include "websocket_handler.h"
#include "rtos_tasks.h"
#include "gnss.h"
#include "logging.h"
#include "navigation.h"

// WebSocket instance (defined in http_server.cpp)
AsyncWebServer webSocketServer(81);
AsyncWebSocket ws("/");

// Timing variables for updates
unsigned long lastWSSensorUpdate = 0;
unsigned long lastWSGPSUpdate = 0;
unsigned long lastWSRTKUpdate = 0;
unsigned long lastWSStatsUpdate = 0;
static bool wasDisconnected = false;

static int mutexWait = 50;

void initWebSocket() {
    // Set event handler
    ws.onEvent(webSocketEventRTOS);
    
    // Add WebSocket to server
    webSocketServer.addHandler(&ws);
    
    // Start WebSocket server
    webSocketServer.begin();
    
    //LOG_DEBUG("AsyncWebSocket server initialized on port 81");
}

// WebSocket event handler for RTOS
void webSocketEventRTOS(AsyncWebSocket *server, AsyncWebSocketClient *client, 
    AwsEventType type, void *arg, uint8_t *data, size_t len) {
    //LOG_DEBUG("webSocketEventRTOS");
    switch (type) {
        case WS_EVT_CONNECT:
            {
                // Client connected
                IPAddress ip = client->remoteIP();
                //LOG_DEBUG("WebSocket client #%u connected from %d.%d.%d.%d", 
                //         client->id(), ip[0], ip[1], ip[2], ip[3]);
                
                // Send initial data to the newly connected client
                sendGPSData(client);
                sendRTKStatus(client);
            }
            break;
        case WS_EVT_DISCONNECT:
            // Client disconnected
            //LOG_DEBUG("WebSocket client #%u disconnected", client->id());
            break;
        case WS_EVT_DATA:
            {
                AwsFrameInfo *info = (AwsFrameInfo*)arg;
                
                // Handle text data (JSON commands)
                if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
                    // Null-terminate the data
                    data[len] = 0;
                    
                    //LOG_DEBUG("WebSocket received text from client, %u, %d", client->id(), len);
                    
                    // Parse the JSON command
                    DynamicJsonDocument doc(JSON_CAPACITY);
                    DeserializationError error = deserializeJson(doc, (char*)data);
                    
                    if (error) {
                        LOG_ERROR("JSON parsing failed, %s", error.c_str());
                        return;
                    }
                    
                    // Handle control messages (manual driving)
                    if (doc.containsKey("control")) {
                        // LOG_DEBUG("Received control command: v=%f, h=%f", 
                        //          (float)doc["control"]["vertical"], 
                        //          (float)doc["control"]["horizontal"]);
                        
                        // Create command struct to send to the control task
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
                                // Queue is full, could add error handling here
                            }
                        }
                    }
                    
                    // Handle autonomous navigation commands
                    else if (doc.containsKey("autonomous")) {
                        String command = doc["autonomous"].as<String>();
                        LOG_NAV("WebSocket received autonomous command, %s", command.c_str());
                        
                        if (command == "start") {
                            // Start autonomous navigation                            
                            // Get parameters
                            bool hasCoordinates = false;
                            
                            float targetPace = doc.containsKey("pace") ? doc["pace"].as<float>() : 1.0f;
                            float targetDistance = doc.containsKey("distance") ? doc["distance"].as<float>() : 0.0f;
                            hasCoordinates = doc.containsKey("lat") && doc.containsKey("lng");
                            
                            // ADD THESE LINES for retrieving coordinates:
                            float latitude = 0.0f;
                            float longitude = 0.0f;
                            
                            LOG_NAV("webSocketEventRTOS data, %.2f, %.2f, %d, %d", 
                                targetPace, targetDistance, hasCoordinates, getWaypointCount());

                            if (hasCoordinates) {
                                // Start navigation to specific coordinates
                                latitude = doc["lat"].as<float>();
                                longitude = doc["lng"].as<float>();
                                startNavigation(targetPace, targetDistance, latitude, longitude);
                            } else if (getWaypointCount() > 0) {
                                // Start navigation using waypoints
                                startWaypointNavigation(targetPace, targetDistance);
                            } else {
                                // No coordinates or waypoints - send error
                                sendErrorMessage("No destination specified. Please enter coordinates or record waypoints.");
                            }
                        } 
                        else if (command == "stop") {
                            // Stop autonomous navigation
                            LOG_NAV("Stop command received");
                            stopNavigation();
                            sendStatusMessage("Navigation stopped");
                        }
                        else if (command == "pause") {
                            // Pause navigation
                            LOG_NAV("Pause command received");
                            pauseNavigation();
                            sendStatusMessage("Navigation paused");
                        }
                        else if (command == "resume") {
                            // Resume navigation
                            LOG_NAV("Resume command received");
                            resumeNavigation();
                            sendStatusMessage("Navigation resumed");
                        }
                    }
                    
                    // Handle waypoint commands
                    else if (doc.containsKey("waypoint")) {
                        String command = doc["waypoint"].as<String>();
                        //LOG_DEBUG("webSocketEventRTOS command, %s", command.c_str());
                        
                        if (command == "record") {
                            // Get current position and record as waypoint

                            LOG_NAV("command = record");
                            if (xSemaphoreTake(gnssMutex, pdMS_TO_TICKS(mutexWait)) == pdTRUE) {
                                float lat = gnssData.latitude;
                                float lon = gnssData.longitude;
                                xSemaphoreGive(gnssMutex);
                        
                                //LOG_NAV("About to call addWaypoint() with lat=%.7f, lon=%.7f", lat, lon);
                                
                                // Add waypoint using current position
                                if (addWaypoint(lat, lon)) {
                                    vTaskDelay(pdMS_TO_TICKS(50));
                                    
                                    // Get waypoint count with proper protection
                                    int count = getWaypointCount();
                                    //LOG_NAV("addWaypoint() succeeded, getWaypointCount() returned %d", count);
                                    
                                    // Get current waypoint index with proper mutex protection
                                    int currentWaypointIndex = 0;
                                    if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(mutexWait)) == pdTRUE) {
                                        currentWaypointIndex = navStatus.currentWaypoint;
                                        xSemaphoreGive(navDataMutex);
                                    }
                                    
                                    // Send confirmation with waypoint info
                                    DynamicJsonDocument response(256);
                                    response["type"] = "waypoint";
                                    response["count"] = count;
                                    response["currentIndex"] = currentWaypointIndex + 1;  // convert 0-index to 1-index
                                    response["lat"] = lat;
                                    response["lng"] = lon;
                                    
                                    String responseStr;
                                    serializeJson(response, responseStr);
                                    //LOG_NAV("Sending waypoint response to client: %s", responseStr.c_str());
                                    client->text(responseStr);
                                } else {
                                    LOG_ERROR("addWaypoint() failed");
                                    sendErrorMessage("Failed to record waypoint (limit reached)");
                                }
                            } else {
                                LOG_ERROR("gnssMutex fail in webSocketEventRTOS");
                                sendErrorMessage("Cannot record waypoint: GPS data unavailable");
                            }
                        }
                        else if (command == "clear") {
                            //LOG_DEBUG("command = clear");
                            // Clear all waypoints
                            clearWaypoints();
                            
                            // Verify count is actually zero with proper protection
                            int count = getWaypointCount();
                            
                            // Send confirmation
                            DynamicJsonDocument response(128);
                            response["type"] = "waypoint";
                            response["count"] = count;
                            
                            // Reset current waypoint index in UI
                            if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(mutexWait)) == pdTRUE) {
                                response["currentIndex"] = navStatus.currentWaypoint + 1;
                                xSemaphoreGive(navDataMutex);
                            } else {
                                response["currentIndex"] = 1; // Default to 1 if mutex can't be taken
                            }
                            
                            String responseStr;
                            serializeJson(response, responseStr);
                            client->text(responseStr);
                            
                            sendStatusMessage("All waypoints cleared");
                        }
                    }
                    
                    // Handle tracking reset command
                    else if (doc.containsKey("tracking") && doc["tracking"].as<String>() == "reset") {
                        //LOG_DEBUG("command = tracking reset");
                        resetNavigationStats();
                        sendStatusMessage("Tracking statistics reset");
                    }
                }
            }
            break;
        case WS_EVT_PONG:
            //LOG_DEBUG("WebSocket received pong from client #%u", client->id());
            break;
            
        case WS_EVT_ERROR:
            LOG_ERROR("WebSocket error from client #%u", client->id());
            break;
    }
}

// Clean up disconnected clients
void cleanupWebSockets() {
    //LOG_DEBUG("cleanupWebSockets");
    ws.cleanupClients();
}

// Send a status message
void sendStatusMessage(const String& message) {
    //LOG_DEBUG("sendStatusMessage");
    DynamicJsonDocument doc(128);
    doc["type"] = "status";
    doc["message"] = message;
    
    String jsonString;
    serializeJson(doc, jsonString);
    ws.textAll(jsonString); // Replace broadcastTXT with textAll
}

// Send an error message
void sendErrorMessage(const String& message) {
    //LOG_DEBUG("sendErrorMessage");
    DynamicJsonDocument doc(128);
    doc["type"] = "error";
    doc["message"] = message;
    
    String jsonString;
    serializeJson(doc, jsonString);
    ws.textAll(jsonString); // Replace broadcastTXT with textAll
}


void WebSocketTask(void *pvParameters) {
    // Initialize task
    //LOG_DEBUG("WebSocketTask");
    unsigned long lastStatusLog = 0;
    unsigned long lastSystemStatsLog = 0;
    unsigned long lastWSSensorUpdate = 0;
    unsigned long lastWSGPSUpdate = 0;
    unsigned long lastWSRTKUpdate = 0;
    unsigned long lastWSStatsUpdate = 0;
    
    // Track previous mode to detect changes
    static bool prevAutonomousMode = false;
    
    // Task loop
    for (;;) {
        unsigned long loopStartTime = millis();
        unsigned long currentTime = millis();
        
        // Log connection status periodically (every 5 seconds)
        if (currentTime - lastStatusLog >= 5000) {
            lastStatusLog = currentTime;
            //LOG_DEBUG("WebSocket clients connected, %d", ws.count());

            // Add this reconnection detection code here
            if (wasDisconnected && ws.count() > 0) {
                //LOG_NAV("WebSocket reconnected after disconnection. Sending status refresh.");
                wasDisconnected = false;
                // Refresh client data
                sendGPSData();
                sendRTKStatus();
                sendNavigationStats();
            }
            
            // Add this disconnection detection code here
            if (ws.count() == 0 && !wasDisconnected) {
                //LOG_NAV("WebSocket disconnected. Will attempt to maintain navigation state.");
                wasDisconnected = true;
            }
        }
        
        // Log system stats periodically (every 30 seconds)
        if (currentTime - lastSystemStatsLog >= 30000) {
            lastSystemStatsLog = currentTime;

            // memory monitoring 
            uint32_t freeHeap = ESP.getFreeHeap();
            // Critical memory warning
            if (freeHeap < 10000) {
                LOG_ERROR("CRITICAL: Low memory condition (%u bytes free)", freeHeap);
            }

            //LOG_DEBUG("free heap bytes, %u", ESP.getFreeHeap());
            //LOG_DEBUG("WebSocketTask high water mark, %u", uxTaskGetStackHighWaterMark(websocketTaskHandle));
            //LOG_DEBUG("GNSSTask high water mark, %u", uxTaskGetStackHighWaterMark(gnssTaskHandle));
            //LOG_DEBUG("ControlTask high water mark, %u", uxTaskGetStackHighWaterMark(controlTaskHandle));
        }
        
        // Clean up disconnected clients periodically
        ws.cleanupClients();
        
        // Check operation mode - autonomous or manual
        bool isAutonomousActive = false;
        bool isPaused = false;
        
        // Get current navigation mode (using minimal mutex time)
        if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(mutexWait)) == pdTRUE) {
            isAutonomousActive = navStatus.autonomousMode;
            isPaused = navStatus.isPaused;
            xSemaphoreGive(navDataMutex);
        } else {
            LOG_ERROR("WebsocketTask navDataMutex timeout");
        }
        
        // Detect mode change
        if (prevAutonomousMode != isAutonomousActive) {
            // Mode has changed - send a complete data update
            sendGPSData();
            sendRTKStatus();
            sendNavigationStats();
            sendSensorData();
            
            // Update previous mode
            prevAutonomousMode = isAutonomousActive;
            
            // Send mode change notification
            DynamicJsonDocument modeDoc(128);
            modeDoc["type"] = "mode";
            modeDoc["autonomous"] = isAutonomousActive;
            String modeJson;
            serializeJson(modeDoc, modeJson);
            ws.textAll(modeJson);
            
            //LOG_NAV("Operation mode changed to: %s", isAutonomousActive ? "autonomous" : "manual");
        }
        
        // Send data based on current mode
        if (isAutonomousActive && !isPaused) {
            // AUTONOMOUS MODE: Only send nav stats during active navigation
            if (currentTime - lastWSStatsUpdate >= WS_STATS_UPDATE_INTERVAL) {
                unsigned long statsStartTime = millis();
                sendNavigationStats();
                unsigned long statsUpdateTime = millis() - statsStartTime;
                if (statsUpdateTime > 20) {
                    //LOG_DEBUG("sendNavigationStats time, %lu", statsUpdateTime);
                }
                lastWSStatsUpdate = currentTime;
            }
        } else {
            // MANUAL MODE: Send all data types
            if (currentTime - lastWSSensorUpdate >= WS_SENSOR_UPDATE_INTERVAL) {
                unsigned long updateStartTime = millis();
                sendSensorData();
                unsigned long updateTime = millis() - updateStartTime;
                if (updateTime > 20) {
                    //LOG_DEBUG("sendSensorData time, %lu", updateTime);
                }
                lastWSSensorUpdate = currentTime;
            }
            
            if (currentTime - lastWSGPSUpdate >= WS_GPS_UPDATE_INTERVAL) {
                unsigned long gpsStartTime = millis();
                sendGPSData();
                unsigned long gpsUpdateTime = millis() - gpsStartTime;
                if (gpsUpdateTime > 20) {
                    //LOG_DEBUG("sendGPSData time, %lu", gpsUpdateTime);
                }
                lastWSGPSUpdate = currentTime;
            }
            
            if (currentTime - lastWSRTKUpdate >= WS_RTK_UPDATE_INTERVAL) {
                unsigned long rtkStartTime = millis();
                sendRTKStatus();
                unsigned long rtkUpdateTime = millis() - rtkStartTime;
                if (rtkUpdateTime > 20) {
                    //LOG_DEBUG("SendRTKStatus time, %lu", rtkUpdateTime);
                }
                lastWSRTKUpdate = currentTime;
            }
            
            if (currentTime - lastWSStatsUpdate >= WS_STATS_UPDATE_INTERVAL) {
                unsigned long statsStartTime = millis();
                sendNavigationStats();
                unsigned long statsUpdateTime = millis() - statsStartTime;
                if (statsUpdateTime > 20) {
                    //LOG_DEBUG("sendNavigationStats time, %lu", statsUpdateTime);
                }
                lastWSStatsUpdate = currentTime;
            }
        }
        
        // Calculate and log the total loop time if significant
        unsigned long loopTime = millis() - loopStartTime;
        if (loopTime > 50) {
            LOG_ERROR("WebSocketTask time, %lu", loopTime);
        }
        
        // Use a short delay to prevent task starvation
        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay
    }
}

// Send GPS data to one client or all clients
void sendGPSData(AsyncWebSocketClient *client) {
    //LOG_DEBUG("sendGPSData");
    unsigned long startTime = millis();
    
    // Variables to store the data we need from gnssData
    uint8_t fixType = 0;
    float latitude = 0, longitude = 0;
    bool validData = false;
    
    // Take mutex ONLY to copy the values we need
    if (xSemaphoreTake(gnssMutex, portMAX_DELAY) == pdTRUE) {
        unsigned long mutexAcquiredTime = millis();
        //LOG_DEBUG("Websocket gnssMutex acq time, %lu", mutexAcquiredTime - startTime);
        
        // Copy only what we need from gnssData
        fixType = gnssData.fixType;
        if (fixType >= 2) {
            latitude = gnssData.latitude;
            longitude = gnssData.longitude;
        }
        validData = true;
        
        // Release mutex immediately after copying data
        unsigned long beforeReleaseTime = millis();
        xSemaphoreGive(gnssMutex);
        //LOG_DEBUG("Websocket gnssMutex hold time, %lu", millis() - beforeReleaseTime);
    } else {
        LOG_ERROR("gnssMutex fail in sendGPSData");
        validData = false;
    }
    
    // Create JSON document for GPS data - AFTER releasing the mutex
    DynamicJsonDocument doc(256);
    doc["type"] = "gps";
    
    // Process the data we copied - all of this happens WITHOUT holding the mutex
    if (validData) {
        if (fixType > 0) {
            // Add fix type description
            String fixDesc;
            switch (fixType) {
                case 1: fixDesc = "1 (Dead Reckoning)"; break;
                case 2: fixDesc = "2 (2D)"; break;
                case 3: fixDesc = "3 (3D)"; break;
                case 4: fixDesc = "4 (GNSS+DR)"; break;
                case 5: fixDesc = "5 (Time Only)"; break;
                default: fixDesc = "Unknown"; break;
            }
            doc["fix"] = fixDesc;
            
            // Add coordinates with precision based on fix type
            if (fixType >= 2) {
                doc["lat"] = String(latitude, 7);
                doc["lng"] = String(longitude, 7);
            }
        } else {
            //LOG_DEBUG("GPS data: No fix available");
            doc["fix"] = "No Fix";
        }
    } else {
        // If we couldn't get the mutex, send minimal data
        doc["fix"] = "Data Unavailable";
    }
    
    // Serialize JSON
    String jsonString;
    serializeJson(doc, jsonString);
    
    // Send to clients
    if (client) {
        // Send to specific client
        client->text(jsonString);
        //LOG_DEBUG("Sent GPS data to client #%u", client->id());
    } else {
        // Broadcast to all clients
        ws.textAll(jsonString);
        //LOG_DEBUG("Broadcast GPS data to all clients");
    }
    
    LOG_DEBUG("sendGPSData time, %lu", millis() - startTime);
}

// Send RTK status to one client or all clients
void sendRTKStatus(AsyncWebSocketClient *client) {
    //LOG_DEBUG("sendRTKStatus");
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
    
    // Serialize JSON
    String jsonString;
    serializeJson(doc, jsonString);
    
    // Send to clients
    if (client) {
        // Send to specific client
        client->text(jsonString);
    } else {
        // Broadcast to all clients
        ws.textAll(jsonString);
    }
}

// Send navigation stats to one client or all clients
void sendNavigationStats(AsyncWebSocketClient *client) {
    //LOG_DEBUG("sendNavigationStats");
    // Get current navigation status
    NavStatus status = getNavStatus();
    
    // Create JSON document
    DynamicJsonDocument doc(256);
    doc["type"] = "navstats";
    doc["totalDistance"] = status.distanceTraveled;
    doc["currentPace"] = status.currentPace;
    doc["averagePace"] = status.averagePace;
    doc["totalTime"] = status.elapsedTime;
    
    // Serialize JSON
    String jsonString;
    serializeJson(doc, jsonString);
    
    // Send to clients
    if (client) {
        // Send to specific client
        client->text(jsonString);
    } else {
        // Broadcast to all clients
        ws.textAll(jsonString);
    }
}

// Send sensor data to one client or all clients
void sendSensorData(AsyncWebSocketClient *client) {
    //LOG_DEBUG("sendSensorData");
    // Create JSON document for sensor data
    // This is placeholder code - you'll need to implement actual sensor reading
    DynamicJsonDocument doc(256);
    doc["type"] = "sensors";
    doc["front"] = 100; // Replace with actual sensor values
    doc["left"] = 100;  // Replace with actual sensor values
    doc["right"] = 100; // Replace with actual sensor values
    
    // Serialize JSON
    String jsonString;
    serializeJson(doc, jsonString);
    
    // Send to clients
    if (client) {
        // Send to specific client
        client->text(jsonString);
    } else {
        // Broadcast to all clients
        ws.textAll(jsonString);
    }
}