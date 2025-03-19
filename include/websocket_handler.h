#ifndef WEBSOCKET_HANDLER_H
#define WEBSOCKET_HANDLER_H

#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "config.h"
#include "route_logger.h"

// WebSocket configuration
const size_t JSON_CAPACITY = 512;  // Adjust based on your data needs
bool webSocketActive = false;
unsigned long lastWSActivity = 0;

// WebSocket update intervals
const unsigned long WS_SENSOR_UPDATE_INTERVAL = 100;   // 100ms for sensor data
const unsigned long WS_GPS_UPDATE_INTERVAL = 200;      // 200ms for GPS data
const unsigned long WS_RTK_UPDATE_INTERVAL = 1000;     // 1000ms for RTK status
const unsigned long WS_STATS_UPDATE_INTERVAL = 500;    // 500ms for navigation stats

// Timing variables for updates
unsigned long lastWSSensorUpdate = 0;
unsigned long lastWSGPSUpdate = 0;
unsigned long lastWSRTKUpdate = 0;
unsigned long lastWSStatsUpdate = 0;

// WebSocket server instance
WebSocketsServer webSocket(81);  // WebSocket server on port 81

// Forward declarations of external variables
extern float lastFrontDist, lastLeftDist, lastRightDist;
extern String lastAvoidanceMessage;
extern float targetLat, targetLon;
extern float waypointLats[], waypointLons[];
extern int waypointCount, currentWaypointIndex;
extern bool followingWaypoints, autonomousMode, destinationReached;
extern volatile float currentLat, currentLon;
extern volatile float currentSpeed;
extern volatile uint8_t currentFixType;
extern bool initialStraightPhase;
extern unsigned long straightPhaseStartTime;
extern float targetPace;
extern float targetDistance;
extern float totalDistance;
extern float currentPace;
extern float averagePace;
extern unsigned long startTime;
extern unsigned long finalElapsedTime;
extern unsigned long lastCommandTime;
extern Servo escServo, steeringServo;
extern unsigned long destinationReachedTime;
extern volatile CorrectionStatus rtcmCorrectionStatus;
extern unsigned long lastReceivedRTCM_ms;
extern unsigned long correctionAge;
extern volatile int carrSoln;
extern volatile double hAcc;
extern WiFiClient ntripClient;

// Forward declarations of all functions
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void processWebSocketCommand(const JsonDocument& doc);
void sendCurrentState(uint8_t clientNum);
void sendGPSData(uint8_t clientNum = 255);
void sendSensorData(uint8_t clientNum = 255);
void sendRTKStatus(uint8_t clientNum = 255);
void sendNavigationStats(uint8_t clientNum = 255);
void sendStatusMessage(const String& message);
void sendErrorMessage(const String& message);
void updateWebSocketClients();

// Forward declarations of external variables
extern bool drivingState;
extern unsigned long lastNonNeutralCommand;

// Initialize WebSocket server
void initWebSockets() {
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started on port 81");
}

// Send a generic status message
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

// Send sensor data via WebSocket
void sendSensorData(uint8_t clientNum) {
  DynamicJsonDocument doc(128);
  doc["type"] = "sensors";
  doc["front"] = lastFrontDist;
  doc["left"] = lastLeftDist;
  doc["right"] = lastRightDist;
  
  // Only include destination reached messages if within the timeout period
  if (lastAvoidanceMessage != "") {
    bool shouldSendMessage = true;
    
    // For destination messages, check if they should be suppressed based on timeout
    if ((lastAvoidanceMessage == "Destination reached" || 
         lastAvoidanceMessage == "Target distance reached") && 
         destinationReached) {
      // Only send the message if the timeout hasn't expired
      if (millis() - destinationReachedTime > DESTINATION_MESSAGE_TIMEOUT) {
        shouldSendMessage = false;
      }
    }
    
    if (shouldSendMessage) {
      doc["message"] = lastAvoidanceMessage;
    }
  }
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  if (clientNum == 255) {
    webSocket.broadcastTXT(jsonString);
  } else {
    webSocket.sendTXT(clientNum, jsonString);
  }
}

// Send GPS data via WebSocket
void sendGPSData(uint8_t clientNum) {
  float distance = 0;
  float bearing = 0;
  
  if (currentFixType && (targetLat != 0 || targetLon != 0)) {
    distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
    bearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
  }
  
  DynamicJsonDocument doc(256);
  doc["type"] = "gps";
  doc["distance"] = distance;
  doc["bearing"] = bearing;
  doc["lat"] = currentLat;
  doc["lng"] = currentLon;
  doc["fix"] = currentFixType;
  doc["destLat"] = targetLat;
  doc["destLng"] = targetLon;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  if (clientNum == 255) {
    webSocket.broadcastTXT(jsonString);
  } else {
    webSocket.sendTXT(clientNum, jsonString);
  }
}

// Send RTK status via WebSocket
void sendRTKStatus(uint8_t clientNum) {
  String correctionStatus;
  unsigned long age = millis() - lastReceivedRTCM_ms;
  
  if (rtcmCorrectionStatus == CORR_FRESH) {
    correctionStatus = "Fresh";
  } else if (rtcmCorrectionStatus == CORR_STALE) {
    correctionStatus = "Stale";
  } else {
    correctionStatus = "None";
  }
  
  DynamicJsonDocument doc(256);
  doc["type"] = "rtk";
  doc["status"] = correctionStatus;
  doc["age"] = age;
  doc["connected"] = ntripClient.connected();
  doc["carrSoln"] = carrSoln;
  doc["hAcc"] = hAcc;
  doc["fixType"] = currentFixType;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  if (clientNum == 255) {
    webSocket.broadcastTXT(jsonString);
  } else {
    webSocket.sendTXT(clientNum, jsonString);
  }
}

// Send navigation stats via WebSocket
void sendNavigationStats(uint8_t clientNum) {
  unsigned long elapsedTime = 0;
  if (autonomousMode) {
    elapsedTime = millis() - startTime;
  } else if (startTime > 0) {
    elapsedTime = finalElapsedTime;
  }
  
  DynamicJsonDocument doc(256);
  doc["type"] = "navstats";
  doc["totalDistance"] = totalDistance;
  doc["currentPace"] = currentPace;
  doc["averagePace"] = averagePace;
  doc["totalTime"] = elapsedTime;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  if (clientNum == 255) {
    webSocket.broadcastTXT(jsonString);
  } else {
    webSocket.sendTXT(clientNum, jsonString);
  }
}

// Send current state to a client
void sendCurrentState(uint8_t clientNum) {
  // Send GPS data
  sendGPSData(clientNum);
  
  // Send sensor data
  sendSensorData(clientNum);
  
  // Send RTK status
  sendRTKStatus(clientNum);
  
  // Send navigation stats if in autonomous mode
  if (autonomousMode) {
    sendNavigationStats(clientNum);
  }
}

// Process received WebSocket command
void processWebSocketCommand(const JsonDocument& doc) {
  if (doc.containsKey("autonomous")) {
    if (doc["autonomous"] == "start") {
      // Get optional parameters from the command
      float lat = doc.containsKey("lat") ? doc["lat"].as<float>() : 0;
      float lng = doc.containsKey("lng") ? doc["lng"].as<float>() : 0;
      float pace = doc.containsKey("pace") ? doc["pace"].as<float>() : DEFAULT_TARGET_PACE;
      float distance = doc.containsKey("distance") ? doc["distance"].as<float>() : DEFAULT_TARGET_DISTANCE;
      
      // Set destination coordinates
      if (lat != 0 && lng != 0) {
        // Use provided coordinates
        targetLat = lat;
        targetLon = lng;
      } else if (waypointCount > 0) {
        // Start with first waypoint
        currentWaypointIndex = 0;
        targetLat = waypointLats[currentWaypointIndex];
        targetLon = waypointLons[currentWaypointIndex];
      } else {
        // No coordinates or waypoints available
        sendErrorMessage("No destination coordinates or waypoints available");
        return;
      }
      
      // Reset navigation tracking variables
      totalDistance = 0.0;
      startTime = millis();
      finalElapsedTime = 0;
      currentPace = 0.0;
      lastPaceUpdate = millis();
      
      // Set pace and distance targets
      targetPace = pace;
      targetDistance = distance;
      
      destinationReached = false;
      autonomousMode = true;
      followingWaypoints = (waypointCount > 0);
      lastAvoidanceMessage = "";
      
      // Initialize straight-line phase
      initialStraightPhase = true;
      straightPhaseStartTime = millis();
      
      // Send confirmation message
      sendStatusMessage("Navigation started");
    }
    else if (doc["autonomous"] == "stop") {
      autonomousMode = false;
      escServo.write(ESC_NEUTRAL);
      steeringServo.write(STEERING_CENTER);
      if (startTime > 0) {
        finalElapsedTime = millis() - startTime;
      }
      sendStatusMessage("Navigation stopped");
    }
  }
  
  // Handle waypoint commands
  else if (doc.containsKey("waypoint")) {
    if (doc["waypoint"] == "record") {
      // Only store if we haven't hit the limit
      if (waypointCount < MAX_WAYPOINTS) {
        waypointLats[waypointCount] = currentLat;
        waypointLons[waypointCount] = currentLon;
        waypointCount++;

        // If route logging enabled:
        if (routeLoggingEnabled) {
          // Get RTK status
          int rtkStatus = 0; // Default to no RTK
          if (carrSoln == 2) rtkStatus = 2; // Fixed
          else if (carrSoln == 1) rtkStatus = 1; // Float
          
          recordRouteWaypoint(currentLat, currentLon, rtkStatus, currentFixType);
        }
        
        DynamicJsonDocument response(128);
        response["type"] = "waypoint";
        response["count"] = waypointCount;
        response["lat"] = currentLat;
        response["lng"] = currentLon;
        
        String jsonString;
        serializeJson(response, jsonString);
        webSocket.broadcastTXT(jsonString);
      } else {
        sendErrorMessage("Maximum waypoint count reached");
      }
    }
    else if (doc["waypoint"] == "clear") {
      waypointCount = 0;
      
      DynamicJsonDocument response(64);
      response["type"] = "waypoint";
      response["count"] = 0;
      
      String jsonString;
      serializeJson(response, jsonString);
      webSocket.broadcastTXT(jsonString);
    }
  }
  
  // Handle tracking reset
  else if (doc.containsKey("tracking") && doc["tracking"] == "reset") {
    resetTracking();
    sendStatusMessage("Tracking reset");
  }
}
// WebSocket event handler function with fast path for control commands
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      // Client disconnected
      if (num == 0) webSocketActive = false;
      break;
      
    case WStype_CONNECTED:
      {
        // Client connected
        IPAddress ip = webSocket.remoteIP(num);
        webSocketActive = true;
        lastWSActivity = millis();
        
        // Send initial state to newly connected client
        sendCurrentState(num);
      }
      break;
      
    case WStype_TEXT:
      {
        lastWSActivity = millis();
        
        // Fast path for control messages - check if this looks like a joystick command
        // This quick check avoids JSON parsing overhead for common control messages
        if (strstr((char*)payload, "control") != NULL) {
          // It looks like a control message - use lightweight parsing
          float normalizedY = 0.0f;
          float normalizedX = 0.0f;
          
          // Fast string parsing technique to extract control values
          // Look for "vertical": and "horizontal": and extract the values
          char* verticalPtr = strstr((char*)payload, "vertical\":");
          char* horizontalPtr = strstr((char*)payload, "horizontal\":");
          
          if (verticalPtr && horizontalPtr) {
            // Move past the property name
            verticalPtr += 10;
            horizontalPtr += 12;
            
            // Parse float values directly
            normalizedY = atof(verticalPtr);
            normalizedX = atof(horizontalPtr);
            
            // Set driving state for non-neutral commands
            const float JOYSTICK_DEADZONE = 0.03f;
            if (abs(normalizedY) > JOYSTICK_DEADZONE || abs(normalizedX) > JOYSTICK_DEADZONE) {
              drivingState = true;
              lastNonNeutralCommand = millis();
            }
            
            // Map joystick to servo values
            int escValue;
            if (abs(normalizedY) < JOYSTICK_DEADZONE) {
              escValue = ESC_NEUTRAL;
            }
            else if (normalizedY > 0) {
              escValue = ESC_NEUTRAL + normalizedY * (ESC_MAX_FWD - ESC_NEUTRAL);
            }
            else {
              escValue = ESC_NEUTRAL + normalizedY * (ESC_NEUTRAL - ESC_MAX_REV);
            }
            
            // Apply ESC deadzone
            if (escValue < ESC_MIN_FWD && escValue > ESC_MIN_REV) {
              escValue = ESC_NEUTRAL;
            }
            
            // Map to steering value - STEERING_CENTER is typically 90
            int steeringValue = STEERING_CENTER + normalizedX * STEERING_MAX;
            
            // Constrain values to valid ranges
            steeringValue = constrain(steeringValue, STEERING_CENTER - STEERING_MAX, STEERING_CENTER + STEERING_MAX);
            escValue = constrain(escValue, ESC_MAX_REV, ESC_MAX_FWD);
            
            // Apply immediately
            escServo.write(escValue);
            steeringServo.write(steeringValue);
            lastCommandTime = millis();
            
            // Skip the rest of processing since we've handled the control message
            return;
          }
        }
        
        // Regular path for other commands - use full JSON parsing
        DynamicJsonDocument doc(JSON_CAPACITY);
        DeserializationError error = deserializeJson(doc, payload);
        
        if (error) {
          return;
        }
        
        // Process other types of commands through the normal path
        processWebSocketCommand(doc);
      }
      break;
  }
}

// Simplified WebSocket client update function that matches main loop behavior
void updateWebSocketClients() {
  unsigned long now = millis();
  
  if (drivingState) {
    // DRIVING STATE: Don't send any data updates while actively driving
    // This matches the main loop which also skips GPS and sensor updates during driving
    // The active joystick commands still work through webSocketEvent
    
    // send a heartbeat every few seconds just to maintain connection
    static unsigned long lastHeartbeat = 0;
    if (now - lastHeartbeat > 3000) { // Every 3 seconds
      lastHeartbeat = now;
      
      // Very minimal status update just to keep connection alive
      DynamicJsonDocument doc(32);
      doc["type"] = "driving";
      String jsonString;
      serializeJson(doc, jsonString);
      webSocket.broadcastTXT(jsonString);
    }
  } 
  else {
    // NON-DRIVING STATE: Send all data as normal
    
    // Send sensor data
    if (now - lastWSSensorUpdate >= WS_SENSOR_UPDATE_INTERVAL) {
      sendSensorData();
      lastWSSensorUpdate = now;
    }
    
    // Send GPS data
    if (now - lastWSGPSUpdate >= WS_GPS_UPDATE_INTERVAL) {
      sendGPSData();
      lastWSGPSUpdate = now;
    }
    
    // Send RTK status
    if (now - lastWSRTKUpdate >= WS_RTK_UPDATE_INTERVAL) {
      sendRTKStatus();
      lastWSRTKUpdate = now;
    }
    
    // Send navigation stats if in autonomous mode
    if (autonomousMode && now - lastWSStatsUpdate >= WS_STATS_UPDATE_INTERVAL) {
      sendNavigationStats();
      lastWSStatsUpdate = now;
    }
  }
}
// // Function to update all WebSocket clients
// void updateWebSocketClients() {
//   unsigned long now = millis();
  
//   // Send sensor data
//   if (now - lastWSSensorUpdate >= WS_SENSOR_UPDATE_INTERVAL) {
//     sendSensorData();
//     lastWSSensorUpdate = now;
//   }
  
//   // Send GPS data
//   if (now - lastWSGPSUpdate >= WS_GPS_UPDATE_INTERVAL) {
//     sendGPSData();
//     lastWSGPSUpdate = now;
//   }
  
//   // Send RTK status
//   if (now - lastWSRTKUpdate >= WS_RTK_UPDATE_INTERVAL) {
//     sendRTKStatus();
//     lastWSRTKUpdate = now;
//   }
  
//   // Send navigation stats if in autonomous mode
//   if (autonomousMode && now - lastWSStatsUpdate >= WS_STATS_UPDATE_INTERVAL) {
//     sendNavigationStats();
//     lastWSStatsUpdate = now;
//   }
// }

#endif // WEBSOCKET_HANDLER_H