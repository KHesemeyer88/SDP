#ifndef WEBSOCKET_HANDLER_H
#define WEBSOCKET_HANDLER_H

#include <WebSocketsServer.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include "config.h"

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

// Buffer for RTCM data
const size_t RTCM_BUFFER_SIZE = 2048;
uint8_t rtcmBuffer[RTCM_BUFFER_SIZE];
size_t rtcmBufferPos = 0;

// WebSocket server and client instances
WebSocketsServer webSocket(81);  // WebSocket server on port 81
WebSocketsClient ntripWebSocket;
bool useNtripWebSocket = false;  // Start with standard HTTP for NTRIP

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
void ntripWebSocketEvent(WStype_t type, uint8_t * payload, size_t length);
bool connectToNtripWebSocket();
void sendNtripConnectionRequest();
void sendGGAToNtripWebSocket(const char* nmeaData);
bool processNtripWebSocketConnection();
void pushGGAWebSocket(NMEA_GGA_data_t *nmeaData);

// Initialize WebSocket server and optionally the NTRIP client
void initWebSockets() {
  // Initialize WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started on port 81");
  
  // Initialize NTRIP WebSocket client if enabled
  if (useNtripWebSocket) {
    connectToNtripWebSocket();
  }
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
  
  if (lastAvoidanceMessage != "") {
    doc["message"] = lastAvoidanceMessage;
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
  // Check for joystick control command
  if (doc.containsKey("control")) {
    float normalizedY = doc["control"]["vertical"];
    float normalizedX = doc["control"]["horizontal"];
    
    // Apply the same control logic as in your HTTP handler
    // Map to ESC values
    int escValue;
    if (abs(normalizedY) < 0.05) { // Small deadzone for joystick
      escValue = ESC_NEUTRAL;
    }
    else if (normalizedY > 0) {
      escValue = ESC_NEUTRAL + normalizedY * (ESC_MAX_FWD - ESC_NEUTRAL);
    }
    else {
      escValue = ESC_NEUTRAL + normalizedY * (ESC_NEUTRAL - ESC_MAX_REV);
    }
    
    // Apply ESC deadzone check
    if (escValue < ESC_MIN_FWD && escValue > ESC_MIN_REV) {
      escValue = ESC_NEUTRAL;
    }
    
    // Map to steering values
    int steeringValue = STEERING_CENTER + normalizedX * STEERING_MAX;
    
    // Apply the values
    escServo.write(escValue);
    steeringServo.write(steeringValue);
    lastCommandTime = millis();
  }
  
  // Handle autonomous mode commands
  else if (doc.containsKey("autonomous")) {
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

// WebSocket event handler function
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WebSocket] #%u Disconnected\n", num);
      if (num == 0) webSocketActive = false;  // Consider main control client disconnected
      break;
      
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[WebSocket] #%u Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
        webSocketActive = true;
        lastWSActivity = millis();
        
        // Send initial state to newly connected client
        sendCurrentState(num);
      }
      break;
      
    case WStype_TEXT:
      {
        Serial.printf("[WebSocket] #%u Received: %s\n", num, payload);
        lastWSActivity = millis();
        
        // Parse the incoming JSON
        DynamicJsonDocument doc(JSON_CAPACITY);
        DeserializationError error = deserializeJson(doc, payload);
        
        if (error) {
          Serial.printf("[WebSocket] Parsing failed: %s\n", error.c_str());
          return;
        }
        
        processWebSocketCommand(doc);
      }
      break;
  }
}

// Function to update all WebSocket clients
void updateWebSocketClients() {
  unsigned long now = millis();
  
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

// Send the NTRIP connection request with authentication
void sendNtripConnectionRequest() {
  // Formulate the NTRIP request
  char serverRequest[512];
  snprintf(serverRequest, sizeof(serverRequest),
         "GET /%s HTTP/1.0\r\n"
         "User-Agent: NTRIP SparkFun u-blox Client v1.0\r\n"
         "Accept: */*\r\n"
         "Connection: Upgrade\r\n"
         "Authorization: Basic %s\r\n"
         "\r\n",
         mountPoint, base64::encode(String(casterUser) + ":" + String(casterUserPW)).c_str());
  
  // Send as text message
  ntripWebSocket.sendTXT(serverRequest);
}

// NTRIP WebSocket event handler
void ntripWebSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("[NTRIP WebSocket] Disconnected");
      break;
      
    case WStype_CONNECTED:
      Serial.println("[NTRIP WebSocket] Connected");
      // After connection, send the connection request with authentication
      sendNtripConnectionRequest();
      break;
      
    case WStype_TEXT:
      // Server response to our connection request
      Serial.printf("[NTRIP WebSocket] Received text: %s\n", payload);
      break;
      
    case WStype_BIN:
      // Handle binary data (RTCM corrections)
      Serial.printf("[NTRIP WebSocket] Received binary data: %u bytes\n", length);
      
      // Update the timestamp for when we last received correction data
      lastReceivedRTCM_ms = millis();
      
      // Push the data directly to the GPS module
      myGPS.pushRawData(payload, length);
      break;
  }
}

// Connect to NTRIP caster using WebSockets
bool connectToNtripWebSocket() {
  Serial.print("Connecting to NTRIP caster via WebSocket: ");
  Serial.println(casterHost);
  
  // Initialize WebSocket client
  ntripWebSocket.begin(casterHost, casterPort, "/"); // Replace with the proper endpoint
  ntripWebSocket.onEvent(ntripWebSocketEvent);
  
  // Use a more frequent ping interval
  ntripWebSocket.setReconnectInterval(5000);
  ntripWebSocket.enableHeartbeat(15000, 3000, 2);
  
  // Wait for connection (with timeout)
  unsigned long connectionStartTime = millis();
  while (!ntripWebSocket.isConnected() && millis() - connectionStartTime < 5000) {
    ntripWebSocket.loop();
    delay(10);
  }
  
  if (!ntripWebSocket.isConnected()) {
    Serial.println("Failed to connect to NTRIP WebSocket server");
    return false;
  }
  
  return true;
}

// Send NMEA GGA data to NTRIP caster
void sendGGAToNtripWebSocket(const char* nmeaData) {
  if (ntripWebSocket.isConnected() && transmitLocation) {
    ntripWebSocket.sendTXT(nmeaData);
  }
}

// Process the NTRIP WebSocket connection
bool processNtripWebSocketConnection() {
  if (!ntripWebSocket.isConnected()) {
    return false;
  }
  
  // Process WebSocket events
  ntripWebSocket.loop();
  
  // Check for timeout
  if ((millis() - lastReceivedRTCM_ms) > maxTimeBeforeHangup_ms) {
    return false;
  }
  
  // Update correction status based on age
  correctionAge = millis() - lastReceivedRTCM_ms;
  if (correctionAge < 5000) { // Less than 5 seconds old
    rtcmCorrectionStatus = CORR_FRESH;
  } else if (correctionAge < 30000) { // Less than 30 seconds old
    rtcmCorrectionStatus = CORR_STALE;
  } else {
    rtcmCorrectionStatus = CORR_NONE;
  }
  
  return true;
}

// Modified callback function for NMEA GGA data
void pushGGAWebSocket(NMEA_GGA_data_t *nmeaData) {
  if (useNtripWebSocket) {
    // Send GGA via WebSocket
    sendGGAToNtripWebSocket((const char *)nmeaData->nmea);
  } else {
    // Original HTTP implementation
    if ((ntripClient.connected() == true) && (transmitLocation == true)) {
      ntripClient.print((const char *)nmeaData->nmea);
    }
  }
}

#endif // WEBSOCKET_HANDLER_H