#include "websocket_handler.h"
#include "rtos_tasks.h"
#include "gnss.h"
#include "logging.h"
#include "navigation.h"
#include <obstacle.h>

// WebSocket instance (defined in http_server.cpp)
AsyncWebServer webSocketServer(81);
AsyncWebSocket ws("/");

//volatile websocket_message_data my_websocket_data;

// Timing variables for updates
unsigned long lastWSSensorUpdate = 0;
unsigned long lastWSGPSUpdate = 0;
unsigned long lastWSRTKUpdate = 0;
unsigned long lastWSStatsUpdate = 0;
static bool wasDisconnected = false;

char currentRouteName[32] = "demo_route";

void initWebSocket() {
    // Set event handler
    ws.onEvent(webSocketEventHandler);
    
    // Add WebSocket to server
    webSocketServer.addHandler(&ws);
    
    // Start WebSocket server
    webSocketServer.begin();
    
    //LOG_DEBUG("AsyncWebSocket server initialized on port 81");
}

// WebSocket event handler for RTOS
void webSocketEventHandler(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    //LOG_DEBUG("webSocketEventRTOS");
    switch (type) {
        case WS_EVT_CONNECT:
            // Client connected
            // IPAddress ip = client->remoteIP();
            // LOG_DEBUG("WebSocket client #%u connected from %d.%d.%d.%d", client->id(), ip[0], ip[1], ip[2], ip[3]);
            
            // // Send initial data to the newly connected client
            sendGPSData();
            sendRTKStatus();
            break;
        case WS_EVT_DISCONNECT:
            // Client disconnected
            //LOG_DEBUG("WebSocket client #%u disconnected", client->id());
            break;
        case WS_EVT_DATA:
            // ----- handle messages -----
            if (len == sizeof(uint8_t)) { // binary size matches uint, its a MESSAGE
                switch (*data) {
                    case MESSAGE_STOP: {
                        // Stop autonomous navigation
                        LOG_NAV("Stop command received");
                        stopNavigation();
                        sendStatusMessage("Navigation stopped");
                        break;
                    }
                    case MESSAGE_RECORD: {
                        // Get current position and record as waypoint
                        LOG_NAV("command = record");
                        if (xSemaphoreTake(gnssMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
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
                                if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                                    currentWaypointIndex = navStatus.currentWaypoint;
                                    xSemaphoreGive(navDataMutex);
                                }
                                
                                Waypoint_data response_message;
                                response_message.count = count;
                                // This isn't used  response["currentIndex"] = currentWaypointIndex + 1;  // convert 0-index to 1-index
                                response_message.lat = lat;
                                response_message.lon = lon;
                                
                                ws.binaryAll((uint8_t*)&response_message, sizeof(response_message));

                            } else {
                                LOG_ERROR("addWaypoint() failed");
                                sendErrorMessage("Failed to record waypoint (limit reached)");
                            }
                        } else {
                            LOG_ERROR("gnssMutex fail in webSocketEventRTOS");
                            sendErrorMessage("Cannot record waypoint: GPS data unavailable");
                        }
                        break;
                    }
                    case MESSAGE_CLEAR: {
                        //LOG_DEBUG("command = clear");
                        // Clear all waypoints
                        clearWaypoints();
                        
                        // Verify count is actually zero with proper protection
                        int count = getWaypointCount();
                        
                        // Send confirmation
                        Waypoint_data response_message;
                        response_message.lat = 0;
                        response_message.lon = 0;
                        response_message.count = count;
                        
                        // // Reset current waypoint index in UI
                        // if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        //     response["currentIndex"] = navStatus.currentWaypoint + 1;
                        //     xSemaphoreGive(navDataMutex);
                        // } else {
                        //     response["currentIndex"] = 1; // Default to 1 if mutex can't be taken
                        // }
                        
                        ws.binaryAll((uint8_t*)&response_message, sizeof(response_message));
                        
                        sendStatusMessage("All waypoints cleared");
                        break;
                    }
                    case MESSAGE_PAUSE: {
                        // Pause navigation
                        LOG_NAV("Pause command received");
                        pauseNavigation();
                        sendStatusMessage("Navigation paused");
                        break;
                    }
                    case MESSAGE_RESUME: {
                        // Resume navigation
                        LOG_NAV("Resume command received");
                        resumeNavigation();
                        sendStatusMessage("Navigation resumed");
                        break;
                    }
                    case MESSAGE_RESET: {
                        // Handle tracking reset command
                        //else if (doc.containsKey("tracking") && doc["tracking"].as<String>() == "reset") {
                        //LOG_DEBUG("command = tracking reset");
                        resetNavigationStats();
                        //sendStatusMessage("Tracking statistics reset");
                        break;
                    }
                }
            } else if (len == sizeof(command_control)){ // Handle control messages (manual driving), binary size matches control struct
                command_control the_message;
                memcpy(&the_message, data, sizeof(the_message)); //copy binary data into struct

                if (the_message.id != COMMAND_CONTROL) {
                    // go to else statement
                    break;
                }
                //Serial.printf("Received command: type=%u value=%.2f\n", my_webpage_data.something, my_webpage_data.something);
                    //if (doc.containsKey("control")) {
                    // LOG_DEBUG("Received control command: v=%f, h=%f", 
                    //          (float)doc["control"]["vertical"], 
                    //          (float)doc["control"]["horizontal"]);

                // Create command struct to send to the control task
                ControlCommand cmd;
                cmd.type = CMD_MANUAL_CONTROL;
                
                // Get joystick values
                float normalizedY = the_message.joystick_y;
                float normalizedX = the_message.joystick_x;
                
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
            } else if (len == sizeof(command_start)) { // Handle autonomous navigation commands
                // Start autonomous navigation                            
                // Get parameters
                command_start the_message;
                memcpy(&the_message, data, sizeof(the_message));

                if (the_message.id != COMMAND_START) {
                    // go to else statement
                    break;
                }
                
                float targetPace = the_message.target_pace;
                float targetDistance = the_message.target_distance;
                
                // ADD THESE LINES for retrieving coordinates:
                float latitude = 0.0f;
                float longitude = 0.0f;
                
                // LOG_NAV("webSocketEventRTOS data, %.2f, %.2f, %d, %d", 
                //     targetPace, targetDistance, hasCoordinates, getWaypointCount());

                if (the_message.lat != 0) {
                    // Start navigation to specific coordinates
                    latitude = the_message.lat;
                    longitude = the_message.lon;
                    startNavigation(targetPace, targetDistance, latitude, longitude);
                } else if (getWaypointCount() > 0) {
                    // Start navigation using waypoints
                    startWaypointNavigation(targetPace - 0.5, targetDistance);
                } else {
                    // No coordinates or waypoints - send error
                    sendErrorMessage("No destination specified. Please enter coordinates or record waypoints.");
                }
            } else if (data[0] == ROUTE_NAME && len > 1 && len < 32) {
                Serial.println("route name");
                // Get current position and record as waypoint
                memcpy(currentRouteName, &data[1], len - 1);
                currentRouteName[len - 1] = '\0'; // null-terminate
                LOG_DEBUG("Received route name: %s", currentRouteName);

                if (xSemaphoreTake(gnssMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    float lat = gnssData.latitude;
                    float lon = gnssData.longitude;
                    xSemaphoreGive(gnssMutex);
                    
                    // Add waypoint using current position
                    if (addWaypoint(lat, lon)) {
                        vTaskDelay(pdMS_TO_TICKS(50));
                        
                        // Get waypoint count with proper protection
                        int count = getWaypointCount();
                        //LOG_NAV("addWaypoint() succeeded, getWaypointCount() returned %d", count);
                        
                        // Get current waypoint index with proper mutex protection
                        int currentWaypointIndex = 0;
                        if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                            currentWaypointIndex = navStatus.currentWaypoint;
                            xSemaphoreGive(navDataMutex);
                        }
                        
                        saveWaypointToNamedRoute(currentRouteName, lat, lon, 8, 8/*carrSoln, fixType*/);

                        Waypoint_data response_message;
                        response_message.count = count;
                        response_message.lat = lat;
                        response_message.lon = lon;
                        
                        ws.binaryAll((uint8_t*)&response_message, sizeof(response_message));
                    }
                }
                break;
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

// WebSocket task implementation
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
        if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            isAutonomousActive = navStatus.autonomousMode;
            isPaused = navStatus.isPaused;
            xSemaphoreGive(navDataMutex);
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
            // Auto_mode response_message;
            // response_message.auto_mode = isAutonomousActive;
            // ws.binaryAll((uint8_t*)&response_message, sizeof(response_message));            
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
            //LOG_DEBUG("WebSocketTask time, %lu", loopTime);
        }
        
        // Use a short delay to prevent task starvation
        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay
    }
}







// Send GPS data to one client or all clients
void sendGPSData(AsyncWebSocketClient *client) {
    //LOG_DEBUG("sendGPSData");
    unsigned long startTime = millis();
    GNSS_data my_data;
        
    // Take mutex ONLY to copy the values we need
    if (xSemaphoreTake(gnssMutex, portMAX_DELAY) == pdTRUE) {
        unsigned long mutexAcquiredTime = millis();
        //LOG_DEBUG("Websocket gnssMutex acq time, %lu", mutexAcquiredTime - startTime);
        
        my_data.gnss_fix_type = gnssData.fixType;
        if (my_data.gnss_fix_type >= 2) {
            my_data.lat = gnssData.latitude;
            my_data.lon = gnssData.longitude;
        }
        
        // Release mutex immediately after copying data
        unsigned long beforeReleaseTime = millis();
        xSemaphoreGive(gnssMutex);
        //LOG_DEBUG("Websocket gnssMutex hold time, %lu", millis() - beforeReleaseTime);
    } else {
        LOG_ERROR("gnssMutex fail in sendGPSData");
    }
    
    
    // Send to clients
    if (client) {
        // Send to specific client
        client->binary((uint8_t*)&my_data, sizeof(my_data));
        //LOG_DEBUG("Sent GPS data to client #%u", client->id());
    } else {
        // Broadcast to all clients
        ws.binaryAll((uint8_t*)&my_data, sizeof(my_data));
        //LOG_DEBUG("Broadcast GPS data to all clients");
    }
    LOG_DEBUG("sendGPSData time, %lu", millis() - startTime);
}

// Send RTK status to one client or all clients
void sendRTKStatus(AsyncWebSocketClient *client) {
    /*
        can use this to make CORR_FRESH itself be seen as 2 by complier
        const int FRESH = 2;
        const uint8_t STALE = 1;
    */

    //LOG_DEBUG("sendRTKStatus");
    struct RTK_status my_data;

    // Take mutex to safely access GNSS data
    if (xSemaphoreTake(gnssMutex, portMAX_DELAY) == pdTRUE) {
        // Add RTK correction status
        switch (rtcmCorrectionStatus) {
            case CORR_FRESH: my_data.RTK_correction_status = 2; break;
            case CORR_STALE: my_data.RTK_correction_status = 1; break;
            case CORR_NONE: my_data.RTK_correction_status = 0; break;
        }
        
        // Add correction age
        my_data.correction_age = correctionAge;
        
        // Take mutex for ntripClient access
        if (xSemaphoreTake(ntripClientMutex, portMAX_DELAY) == pdTRUE) {
            // Add RTK connection status
            my_data.connection_status = ntripClient.connected();
            xSemaphoreGive(ntripClientMutex);
        }
        
        // Add RTK solution status (carrier solution)
        my_data.RTK_carrier_solution = gnssData.carrSoln;
        
        // Add horizontal accuracy
        my_data.h_accuracy = gnssData.hAcc;
        
        // Add fix type
        my_data.gnss_fix_type = gnssData.fixType;
        
        // Release GNSS mutex
        xSemaphoreGive(gnssMutex);
    }
    

    
    // Send to clients
    if (client) {
        // Send to specific client
        client->binary((uint8_t*)&my_data, sizeof(my_data));
        //LOG_DEBUG("Sent GPS data to client #%u", client->id());
    } else {
        // Broadcast to all clients
        ws.binaryAll((uint8_t*)&my_data, sizeof(my_data));
        //LOG_DEBUG("Broadcast GPS data to all clients");
    }
}

// Send navigation stats to one client or all clients
void sendNavigationStats(AsyncWebSocketClient *client) {
    //LOG_DEBUG("sendNavigationStats");
    // Get current navigation status
    NavStatus status = getNavStatus();
    Nav_stats my_data;
    
    my_data.total_distance = status.distanceTraveled;
    my_data.current_pace = status.currentPace;
    my_data.average_pace = status.averagePace;
    my_data.total_time = status.elapsedTime;
    
    // Send to clients
    if (client) {
        // Send to specific client
        client->binary((uint8_t*)&my_data, sizeof(my_data));
        //LOG_DEBUG("Sent GPS data to client #%u", client->id());
    } else {
        // Broadcast to all clients
        ws.binaryAll((uint8_t*)&my_data, sizeof(my_data));
        //LOG_DEBUG("Broadcast GPS data to all clients");
    }
}

// Send sensor data to one client or all clients
void sendSensorData(AsyncWebSocketClient *client) {
    //LOG_DEBUG("sendSensorData");
    // This is placeholder code - you'll need to implement actual sensor reading

    Sensor_data my_data;
    my_data.lidar_front = lastFrontDist; // Replace with actual sensor values
    //my_data.lidar_left = 100;  // Replace with actual sensor values
    //my_data.lidar_right = 100; // Replace with actual sensor values
    
    // Send to clients
    if (client) {
        // Send to specific client
        client->binary((uint8_t*)&my_data, sizeof(my_data));
        //LOG_DEBUG("Sent GPS data to client #%u", client->id());
    } else {
        // Broadcast to all clients
        ws.binaryAll((uint8_t*)&my_data, sizeof(my_data));
        //LOG_DEBUG("Broadcast GPS data to all clients");
    }
}

// Send a status message
void sendStatusMessage(const String& message) {
    // //LOG_DEBUG("sendStatusMessage");
    // DynamicJsonDocument doc(128);
    // doc["type"] = "status";
    // doc["message"] = message;
    
    // String jsonString;
    // serializeJson(doc, jsonString);
    // ws.textAll(jsonString); // Replace broadcastTXT with textAll
}

// Send an error message
void sendErrorMessage(const String& message) {
    // //LOG_DEBUG("sendErrorMessage");
    // DynamicJsonDocument doc(128);
    // doc["type"] = "error";
    // doc["message"] = message;
    
    // String jsonString;
    // serializeJson(doc, jsonString);
    // ws.textAll(jsonString); // Replace broadcastTXT with textAll
}