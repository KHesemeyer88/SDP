#ifndef WEBHANDLERS_H
#define WEBHANDLERS_H

#include <WebServer.h>
#include "config.h"
#include "webpage.h"
#include "navigation.h"  // For resetTracking()

// Forward declarations for calculateDistance and calculateBearing
float calculateDistance(float lat1, float lon1, float lat2, float lon2);
float calculateBearing(float lat1, float lon1, float lat2, float lon2);

extern WebServer server;
extern String lastAvoidanceMessage;
extern float lastFrontDist, lastLeftDist, lastRightDist;
extern float targetLat, targetLon;
extern float waypointLats[], waypointLons[];
extern int waypointCount, currentWaypointIndex;
extern bool autonomousMode, followingWaypoints, destinationReached;
extern unsigned long destinationReachedTime;
extern SFE_UBLOX_GNSS myGPS;
extern Servo escServo, steeringServo;
//extern int waypointLoopCount;
//extern int targetLoopCount;
extern float targetPace;
extern float targetDistance;
extern float totalDistance;
extern unsigned long finalElapsedTime;
extern unsigned long startTime;
extern unsigned long totalTimeMs;
extern float currentPace;
extern float averagePace;
extern unsigned long lastCommandTime;
// Straight phase variables
extern bool initialStraightPhase;
extern unsigned long straightPhaseStartTime;

// Function to set up all web server routes
void setupWebServerRoutes() {
    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", webPage);
    });

    server.on("/control", HTTP_GET, []() {
        String verticalStr = server.arg("vertical");
        String horizontalStr = server.arg("horizontal");

        if (verticalStr != "" && horizontalStr != "") {
            float normalizedY = verticalStr.toFloat();
            float normalizedX = horizontalStr.toFloat();
            
            // Map to ESC values
            int escValue;
            if (abs(normalizedY) < 0.05) { // Small deadzone for joystick itself
                escValue = ESC_NEUTRAL;
            }
            else if (normalizedY > 0) {
                escValue = ESC_NEUTRAL + normalizedY * (ESC_MAX_FWD - ESC_NEUTRAL);
            }
            else {
                escValue = ESC_NEUTRAL + normalizedY * (ESC_NEUTRAL - ESC_MAX_REV);
            }
            
            // Apply ESC deadzone check (separate from joystick deadzone)
            if (escValue < ESC_MIN_FWD && escValue > ESC_MIN_REV) {
                escValue = ESC_NEUTRAL;
            }
            
            // Map to steering values
            int steeringValue = STEERING_CENTER + normalizedX * STEERING_MAX;
            
            // Apply the values
            escServo.write(escValue);
            steeringServo.write(steeringValue);
            lastCommandTime = millis();
            server.send(200, "text/plain", "OK");
        } else {
            server.send(400, "text/plain", "Missing parameters");
        }
    });

    server.on("/sensors", HTTP_GET, []() {
        String json = "{\"front\":" + String(lastFrontDist, 1) +
                      ",\"left\":" + String(lastLeftDist, 1) +
                      ",\"right\":" + String(lastRightDist, 1) +
                      ",\"message\":\"" + lastAvoidanceMessage + "\"}";
        if (lastAvoidanceMessage != "Destination reached") {
            lastAvoidanceMessage = "";
        }
        server.send(200, "application/json", json);
    });

    server.on("/gps", HTTP_GET, []() {
        float distance = 0;
        float bearing = 0;
        
        if (currentFixType && (targetLat != 0 || targetLon != 0)) {
            distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
            bearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
        }

        String json = "{\"distance\":" + String(distance, 1) +
                      ",\"bearing\":" + String(bearing, 1) +
                      ",\"lat\":" + String(currentLat, 6) +
                      ",\"lng\":" + String(currentLon, 6) +
                      ",\"fix\":\"" + String(currentFixType) + "\"" +
                      ",\"destLat\":" + String(targetLat, 6) +
                      ",\"destLng\":" + String(targetLon, 6) +
                      "}";
        server.send(200, "application/json", json);
    });

    server.on("/setDestination", HTTP_GET, []() {
        String lat = server.arg("lat");
        String lng = server.arg("lng");
        String pace = server.arg("pace");
        String distance = server.arg("distance");
        
        // Set destination coordinates
        if (lat != "" && lng != "") {
            // Use manually entered coordinates
            targetLat = lat.toFloat();
            targetLon = lng.toFloat();
        } else if (waypointCount > 0) {
            // Start with first waypoint
            currentWaypointIndex = 0;
            targetLat = waypointLats[currentWaypointIndex];
            targetLon = waypointLons[currentWaypointIndex];
        } else {
            server.send(400, "text/plain", "No destination coordinates provided or waypoints recorded");
            return;
        }
        
        // Reset navigation tracking variables
        totalDistance = 0.0;
        startTime = millis();  // Set the start time when autonomous mode begins
        finalElapsedTime = 0;
        currentPace = 0.0;
        lastPaceUpdate = millis();
        
        // Set pace and distance targets
        if (pace != "") {
            targetPace = pace.toFloat();
        }
        else {
            targetPace = DEFAULT_TARGET_PACE;
        }
        if (distance != "") {
            targetDistance = distance.toFloat();
        }
        else {
            targetDistance = DEFAULT_TARGET_DISTANCE;
        }

        destinationReached = false;
        autonomousMode = true;
        followingWaypoints = (waypointCount > 0);
        lastAvoidanceMessage = "";
        // Initialize straight-line phase
        initialStraightPhase = true;
        straightPhaseStartTime = millis();
        server.send(200, "text/plain", "Navigation started");
    });
    
    
    server.on("/stopNavigation", HTTP_GET, []() {
        autonomousMode = false;
        escServo.write(ESC_NEUTRAL);
        steeringServo.write(STEERING_CENTER);
        if (startTime > 0) {
            finalElapsedTime = millis() - startTime;
        }
        server.send(200, "text/plain", "Navigation stopped");
    });


    server.on("/fusionStatus", HTTP_GET, []() {
        //Serial.println("Fusion status requested");
        myGPS.setNavigationFrequency(1);
        String fusionStatus = "Failed to get fusion data"; // Can't seem to get fusion status at a higher frequency, don't know why
        
        if (!myGPS.getEsfInfo()) {
            while(1) { // CHANGE THIS TO TIME OUT IN CASE IT NEVER HAPPENS!!!!!!!!!!!!!!!!!!
                if (myGPS.getEsfInfo()) {
                    uint8_t fusionMode = myGPS.packetUBXESFSTATUS->data.fusionMode;
                    
                    
                    switch(fusionMode) {
                        case 0: fusionStatus = "Initializing"; break;
                        case 1: fusionStatus = "Calibrated"; break;
                        case 2: fusionStatus = "Suspended"; break;
                        case 3: fusionStatus = "Disabled"; break;
                    }
                    break;
                }
            }
        }

        String json = "{\"status\":\"" + fusionStatus + "\"}";
        server.send(200, "application/json", json);
        myGPS.setNavigationFrequency(10);
    });
    
    server.on("/currentWP", HTTP_GET, []() {
        // Only store if we haven't hit the limit
        if (waypointCount < MAX_WAYPOINTS) {
            waypointLats[waypointCount] = currentLat;
            waypointLons[waypointCount] = currentLon;
            waypointCount++;
        }

        String json = "{\"lat\":" + String(currentLat, 6) + 
                      ",\"lng\":" + String(currentLon, 6) + 
                      ",\"count\":" + String(waypointCount) + "}";
        
        server.send(200, "application/json", json);
    });


    server.on("/clearWaypoints", HTTP_GET, []() {
        waypointCount = 0;
        String json = "{\"count\":" + String(waypointCount) + "}";
        server.send(200, "application/json", json);
    });
    
    
    server.on("/navstats", HTTP_GET, []() {
        unsigned long elapsedTime = 0;
        if (autonomousMode) {
            elapsedTime = millis() - startTime;
        } else if (startTime > 0) {
            elapsedTime = finalElapsedTime;
        }
        
        String json = "{\"totalDistance\":" + String(totalDistance, 1) +
                      ",\"currentPace\":" + String(currentPace, 2) +
                      ",\"averagePace\":" + String(averagePace, 2) +
                      ",\"totalTime\":" + String(elapsedTime) + "}";
        
        server.send(200, "application/json", json);
    });
    
    server.on("/resetTracking", HTTP_GET, []() {
        resetTracking();
        server.send(200, "text/plain", "Tracking reset");
    });

    server.on("/messages", HTTP_GET, []() {
        String json = "{\"message\":\"" + lastAvoidanceMessage + "\"}";
        if (lastAvoidanceMessage != "Destination reached") {
            lastAvoidanceMessage = "";
        }
        server.send(200, "application/json", json);
    });

    server.on("/status", HTTP_GET, []() {
        String correctionStatus;
        unsigned long age = millis() - lastReceivedRTCM_ms;
        
        if (rtcmCorrectionStatus == CORR_FRESH) {
            correctionStatus = "Fresh";
        } else if (rtcmCorrectionStatus == CORR_STALE) {
            correctionStatus = "Stale";
        } else {
            correctionStatus = "None";
        }
        
        String json = "{\"rtk\":{";
        json += "\"status\":\"" + correctionStatus + "\",";
        json += "\"age\":" + String(age) + ",";
        json += "\"connected\":" + String(ntripClient.connected() ? "true" : "false") + ",";
        json += "\"carrSoln\":" + String(carrSoln) + ",";
        json += "\"hAcc\":" + String(hAcc, 2) + ",";
        json += "\"fixType\":" + String(currentFixType);
        json += "}}";
        
        server.send(200, "application/json", json);
    });
}

#endif // WEBHANDLERS_H