#include "https_and_ws.h"
#include "rtos_tasks.h"
#include "gnss.h"
#include "logging.h"
#include "navigation.h"
#include <obstacle.h>
#include "webpage.h" 
#include <istream>

SSLCert* cert = new SSLCert(
    (unsigned char*)server_cert, strlen(server_cert),
    (unsigned char*)server_key, strlen(server_key)
);
HTTPSServer secureServer(cert);

GNSS_data my_gnss_data;
RTK_status my_rtk_status;
Nav_stats my_nav_stats;
Sensor_data my_sensor_data;
Waypoint_data my_waypoint_data;
phone_position my_phone_position;

// Timing variables for updates
unsigned long lastWSSensorUpdate = 0;
unsigned long lastWSGPSUpdate = 0;
unsigned long lastWSRTKUpdate = 0;
unsigned long lastWSStatsUpdate = 0;
//static bool wasDisconnected = false;

using namespace httpsserver;

std::vector<GNSSWebSocketHandler*> activeClients;

httpsserver::WebsocketHandler* GNSSWebSocketHandler::create() {
    auto* handler = new GNSSWebSocketHandler();
    if (activeClients.size() < MAX_CLIENTS) {
        activeClients.push_back(handler);
    }
        return handler;
}

       
void GNSSWebSocketHandler::onMessage(WebsocketInputStreambuf* input) {
    // Wrap the streambuf with istream
    std::istream is(input);
    uint8_t id;
    is.read((char*)&id, sizeof(id));
    // std::ostringstream oss;
    // oss << input;
    // std::string msg = oss.str();
    // size_t len = input->size();
    // const uint8_t* data = (const uint8_t*)input->data();
    //size_t len = message->length();
    //const uint8_t* data = (const uint8_t*)message->data();

    // ----- handle messages -----
    if (id == MESSAGE_STOP && is) { 
        // Stop autonomous navigation
        //LOG_NAV("Stop command received");
        stopNavigation();
        sendStatusMessage("Navigation stopped");
    } else if (id == MESSAGE_RECORD && is) {
        // Get current position and record as waypoint
        //LOG_NAV("command = record");
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
                
                my_waypoint_data.count = count;
                // This isn't used  response["currentIndex"] = currentWaypointIndex + 1;  // convert 0-index to 1-index
                my_waypoint_data.lat = lat;
                my_waypoint_data.lon = lon;
                
                // Send to clients
                for (auto* client : activeClients) {
                    client->sendWAYPOINTBinary();
                }

            } else {
                //LOG_ERROR("addWaypoint() failed");
                sendErrorMessage("Failed to record waypoint (limit reached)");
            }
        } else {
            //LOG_ERROR("gnssMutex fail in webSocketEventRTOS");
            sendErrorMessage("Cannot record waypoint: GPS data unavailable");
        }
    } else if (id == MESSAGE_CLEAR && is) {
        //LOG_DEBUG("command = clear");
        // Clear all waypoints
        clearWaypoints();
        
        // Verify count is actually zero with proper protection
        int count = getWaypointCount();
        
        // Send confirmation
        my_waypoint_data.count = count;
        my_waypoint_data.lat = 0;
        my_waypoint_data.lon = 0;
        // Send to clients
        for (auto* client : activeClients) {
            client->sendWAYPOINTBinary();
        }
        
        sendStatusMessage("All waypoints cleared");
    } else if (id == MESSAGE_PAUSE && is) {
        // Pause navigation
        //LOG_NAV("Pause command received");
        pauseNavigation();
        sendStatusMessage("Navigation paused");
    } else if (id == MESSAGE_RESUME && is) {
        // Resume navigation
        //LOG_NAV("Resume command received");
        resumeNavigation();
        sendStatusMessage("Navigation resumed");
    } else if (id == MESSAGE_RESET && is) {
        resetNavigationStats();   
    } else if (id == COMMAND_CONTROL_ID && is){ // Handle control messages (manual driving), binary size matches control struct
        //command_control the_message;
        //memcpy(&the_message, data, sizeof(the_message)); //copy binary data into struct
        //Serial.printf("Received command: type=%u value=%.2f\n", my_webpage_data.something, my_webpage_data.something);
            //if (doc.containsKey("control")) {
            // LOG_DEBUG("Received control command: v=%f, h=%f", 
            //          (float)doc["control"]["vertical"], 
            //          (float)doc["control"]["horizontal"]);

        // Create command struct to send to the control task
        command_control the_message;
        the_message.id = id;
        is.read((char*)&the_message.joystick_y, sizeof(the_message.joystick_y));
        is.read((char*)&the_message.joystick_x, sizeof(the_message.joystick_x));
        // if (is) {
        // }




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
    } else if (id == COMMAND_START_ID && is) { // Handle autonomous navigation commands
        // Start autonomous navigation                            
        // Get parameters
        command_start the_message;
        //memcpy(&the_message, data, sizeof(the_message));

        the_message.id = id;
        is.read((char*)&the_message.target_pace, sizeof(the_message.target_pace));
        is.read((char*)&the_message.target_distance, sizeof(the_message.target_distance));
        is.read((char*)&the_message.lat, sizeof(the_message.lat));
        is.read((char*)&the_message.lon, sizeof(the_message.lon));


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
    } else if (id == PHONE_POSITION_ID && is) {
        phone_position data;
        data.id = id;
        is.read((char*)&data.phone_lat, sizeof(data.phone_lat));
        is.read((char*)&data.phone_lon, sizeof(data.phone_lon));
        is.read((char*)&data.phone_speed, sizeof(data.phone_speed));

        if (is) {
            my_phone_position = data;
            Serial.printf("[PHONE] lat=%.6f, lon=%.6f, speed=%.2f\n", data.phone_lat, data.phone_lon, data.phone_speed);
        }
    }
}

void GNSSWebSocketHandler::onClose() {
    activeClients.erase(std::remove(activeClients.begin(), activeClients.end(), this), activeClients.end());
}

void GNSSWebSocketHandler::sendGNSSBinary() {
    send((uint8_t*)&my_gnss_data, sizeof(my_gnss_data), SEND_TYPE_BINARY);
}
void GNSSWebSocketHandler::sendRTKBinary() {
    send((uint8_t*)&my_rtk_status, sizeof(my_rtk_status), SEND_TYPE_BINARY);
}
void GNSSWebSocketHandler::sendNAVBinary() {
    send((uint8_t*)&my_nav_stats, sizeof(my_nav_stats), SEND_TYPE_BINARY);
}
void GNSSWebSocketHandler::sendSENSORBinary() {
    send((uint8_t*)&my_sensor_data, sizeof(my_sensor_data), SEND_TYPE_BINARY);
}
void GNSSWebSocketHandler::sendWAYPOINTBinary() {
    send((uint8_t*)&my_waypoint_data, sizeof(my_waypoint_data), SEND_TYPE_BINARY);
}


void initWebSocket() {
    // Register page route
    secureServer.registerNode(new ResourceNode("/", "GET", &handleRoot));
    secureServer.registerNode(new WebsocketNode("/ws", &GNSSWebSocketHandler::create));
    secureServer.start();
}

// Serve HTML at root / initHTTPSServer / prev initHttpServer
void handleRoot(HTTPRequest* req, HTTPResponse* res) {
    res->setHeader("Content-Type", "text/html");
    res->println(webPage);
}

// HTTP server task function
void HttpServerTask(void *pvParameters) {
    // Initialize task
    //LOG_DEBUG("HTTP task start");
    
    // Initialize server routes and start the server
    Serial.printf("Starting HTTPS server\n");
    //secureServer.start();
    //handleRoot(req, res);
     for (;;) {        
        secureServer.loop();
        // Task heartbeat
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Clean up disconnected clients
void cleanupWebSockets() {
    //LOG_DEBUG("cleanupWebSockets");
    //ws.cleanupClients();
}

// WebSocket task implementation
void WebSocketTask(void *pvParameters) {
    // Initialize task
    //LOG_DEBUG("WebSocketTask");
    //unsigned long lastStatusLog = 0;
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
        // if (currentTime - lastStatusLog >= 5000) {
        //     lastStatusLog = currentTime;
            //LOG_DEBUG("WebSocket clients connected, %d", ws.count());

            // // Add this reconnection detection code here
            // if (wasDisconnected && ws.count() > 0) {
            //     //LOG_NAV("WebSocket reconnected after disconnection. Sending status refresh.");
            //     wasDisconnected = false;
            //     // Refresh client data
                sendGPSData();
                sendRTKStatus();
                sendNavigationStats();
            //}
            
            // // Add this disconnection detection code here
            // if (ws.count() == 0 && !wasDisconnected) {
            //     //LOG_NAV("WebSocket disconnected. Will attempt to maintain navigation state.");
            //     wasDisconnected = true;
            // }
        //}
        
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
        //ws.cleanupClients();
        
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
void sendGPSData() {
    //LOG_DEBUG("sendGPSData");
    unsigned long startTime = millis();

    // Take mutex ONLY to copy the values we need
    if (xSemaphoreTake(gnssMutex, portMAX_DELAY) == pdTRUE) {
        unsigned long mutexAcquiredTime = millis();
        //LOG_DEBUG("Websocket gnssMutex acq time, %lu", mutexAcquiredTime - startTime);
        
        my_gnss_data.gnss_fix_type = gnssData.fixType;
        if (my_gnss_data.gnss_fix_type >= 2) {
            my_gnss_data.lat = gnssData.latitude;
            my_gnss_data.lon = gnssData.longitude;
        }
        
        // Release mutex immediately after copying data
        unsigned long beforeReleaseTime = millis();
        xSemaphoreGive(gnssMutex);
        //LOG_DEBUG("Websocket gnssMutex hold time, %lu", millis() - beforeReleaseTime);
    } else {
        LOG_ERROR("gnssMutex fail in sendGPSData");
    }
    
    // Send to clients
    for (auto* client : activeClients) {
        client->sendGNSSBinary();
    }

    LOG_DEBUG("sendGPSData time, %lu", millis() - startTime);
}

// Send RTK status to one client or all clients
void sendRTKStatus() {
    /*
        can use this to make CORR_FRESH itself be seen as 2 by complier
        const int FRESH = 2;
        const uint8_t STALE = 1;
    */

    //LOG_DEBUG("sendRTKStatus");

    // Take mutex to safely access GNSS data
    if (xSemaphoreTake(gnssMutex, portMAX_DELAY) == pdTRUE) {
        // Add RTK correction status
        switch (rtcmCorrectionStatus) {
            case CORR_FRESH: my_rtk_status.RTK_correction_status = 2; break;
            case CORR_STALE: my_rtk_status.RTK_correction_status = 1; break;
            case CORR_NONE: my_rtk_status.RTK_correction_status = 0; break;
        }
        
        // Add correction age
        my_rtk_status.correction_age = correctionAge;
        
        // Take mutex for ntripClient access
        if (xSemaphoreTake(ntripClientMutex, portMAX_DELAY) == pdTRUE) {
            // Add RTK connection status
            my_rtk_status.connection_status = ntripClient.connected();
            xSemaphoreGive(ntripClientMutex);
        }
        
        // Add RTK solution status (carrier solution)
        my_rtk_status.RTK_carrier_solution = gnssData.carrSoln;
        
        // Add horizontal accuracy
        my_rtk_status.h_accuracy = gnssData.hAcc;
        
        // Add fix type
        my_rtk_status.gnss_fix_type = gnssData.fixType;
        
        // Release GNSS mutex
        xSemaphoreGive(gnssMutex);
    }
    
    // Send to clients
    for (auto* client : activeClients) {
        client->sendRTKBinary();
    }
}

// Send navigation stats to one client or all clients
void sendNavigationStats() {
    //LOG_DEBUG("sendNavigationStats");
    // Get current navigation status
    NavStatus status = getNavStatus();
    
    my_nav_stats.total_distance = status.distanceTraveled;
    my_nav_stats.current_pace = status.currentPace;
    my_nav_stats.average_pace = status.averagePace;
    my_nav_stats.total_time = status.elapsedTime;
    
    // Send to clients
    for (auto* client : activeClients) {
        client->sendNAVBinary();
    }
}

// Send sensor data to one client or all clients
void sendSensorData() {
    //LOG_DEBUG("sendSensorData");
    // This is placeholder code - you'll need to implement actual sensor reading

    my_sensor_data.lidar_front = lastFrontDist; // Replace with actual sensor values
    //my_data.lidar_left = 100;  // Replace with actual sensor values
    //my_data.lidar_right = 100; // Replace with actual sensor values
    
    // Send to clients
    for (auto* client : activeClients) {
        client->sendSENSORBinary();
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