#include "websocket_handler.h"
#include "rtos_tasks.h"

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
            Serial.printf("[WebSocket] Client #%u disconnected\n", num);
            break;
            
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket.remoteIP(num);
                Serial.printf("[WebSocket] Client #%u connected from %d.%d.%d.%d\n", 
                              num, ip[0], ip[1], ip[2], ip[3]);
                
                // Send initial state to newly connected client
                // (To be implemented later when we add sensor/GPS functionality)
            }
            break;
            
        case WStype_TEXT:
            {
                // Parse the JSON command
                DynamicJsonDocument doc(512);
                DeserializationError error = deserializeJson(doc, payload);
                
                if (error) {
                    Serial.println("JSON parsing failed!");
                    return;
                }
                
                // Fast path for control messages (manual driving)
                if (doc.containsKey("control")) {
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
                            // Queue full - this is just a log, no action needed as we'll get another command soon
                            Serial.println("Command queue full");
                        }
                    }
                    
                    // Command processed, return immediately
                    return;
                }
                
                // Other command types will be implemented later
                // For now, we're just focusing on manual driving
            }
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
    Serial.println("WebSocket server started on port 81");
    
    // Task loop
    for (;;) {
        // Handle WebSocket events
        webSocket.loop();
        
        // Send periodic updates
        unsigned long currentTime = millis();
        
        // Send sensor data if interval elapsed - implement later
        /*
        if (currentTime - lastWSSensorUpdate >= WS_SENSOR_UPDATE_INTERVAL) {
            // sendSensorData();
            lastWSSensorUpdate = currentTime;
        }
        
        // Send GPS data if interval elapsed - implement later
        if (currentTime - lastWSGPSUpdate >= WS_GPS_UPDATE_INTERVAL) {
            // sendGPSData();
            lastWSGPSUpdate = currentTime;
        }
        */
        
        // Small yield to allow other tasks to run
        vTaskDelay(1);
    }
}

// These function implementations will be added later when we implement
// sensor reading and GPS functionality

void sendSensorData(uint8_t clientNum) {
    // To be implemented
}

void sendGPSData(uint8_t clientNum) {
    // To be implemented
}

void sendRTKStatus(uint8_t clientNum) {
    // To be implemented  
}

void sendNavigationStats(uint8_t clientNum) {
    // To be implemented
}