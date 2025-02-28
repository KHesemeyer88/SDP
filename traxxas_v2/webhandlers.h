// webhandlers.h
#ifndef WEBHANDLERS_H
#define WEBHANDLERS_H

#include <WebServer.h>
#include "config.h" // For constants and shared variables
#include "webpage.h"

// Forward declaration of functions used in handlers
float calculateDistance(float lat1, float lon1, float lat2, float lon2);
float calculateBearing(float lat1, float lon1, float lat2, float lon2);

// This extern declaration tells the compiler these variables exist somewhere else
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
extern unsigned long lastUpdateTime;
extern int waypointLoopCount;
extern int targetLoopCount;
extern float targetPace;
extern float targetDistance;
extern float totalDistance;
extern unsigned long totalTimeMs;
extern float currentPace;

// Function to set up all web server routes
void setupWebServerRoutes() {
    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", webPage);
    });

    server.on("/control", HTTP_GET, []() {
        String speedStr = server.arg("speed");
        String angleStr = server.arg("angle");

        if (speedStr != "" && angleStr != "") {
            int speed = speedStr.toInt();
            int angle = angleStr.toInt();
            
            // Convert -255 to 255 speed range to ESC range
            int escValue;
            if (abs(speed) < 20) {  // Small deadzone for neutral
                escValue = ESC_NEUTRAL;
            } else if (speed > 0) {
                escValue = map(speed, 20, 255, ESC_MIN_FWD, ESC_MAX_FWD);
            } else {
                escValue = map(speed, -255, -20, ESC_MAX_REV, ESC_MIN_REV);
            }
            
            escServo.write(escValue);
            steeringServo.write(angle);
            
            lastUpdateTime = millis();
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
        float currentLat = myGPS.getLatitude() / 10000000.0;
        float currentLon = myGPS.getLongitude() / 10000000.0;
        
        float distance = 0;
        float bearing = 0;
        unsigned char fixType = myGPS.getFixType();
        
        if (fixType && (targetLat != 0 || targetLon != 0)) {
            distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
            bearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
        }

        String json = "{\"distance\":" + String(distance, 1) +
                      ",\"bearing\":" + String(bearing, 1) +
                      ",\"lat\":" + String(currentLat, 6) +
                      ",\"lng\":" + String(currentLon, 6) +
                      ",\"fix\":\"" + String(fixType) + "\"" +
                      ",\"destLat\":" + String(targetLat, 6) +
                      ",\"destLng\":" + String(targetLon, 6) +
                      "}";
        server.send(200, "application/json", json);
    });

    server.on("/setDestination", HTTP_GET, []() {
        String lat = server.arg("lat");
        String lng = server.arg("lng");
        String loops = server.arg("loops");
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
        waypointLoopCount = 0;
        totalDistance = 0.0;
        totalTimeMs = millis();
        currentPace = 0.0;
        lastPaceUpdate = millis();
        
        // Get current position for initial segment distance calculation
        float currentLat = myGPS.getLatitude() / 10000000.0;
        float currentLon = myGPS.getLongitude() / 10000000.0;
        lastSegmentDistance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
        
        // Set target values from parameters
        if (loops != "") {
            targetLoopCount = loops.toInt();
        } else {
            targetLoopCount = DEFAULT_LOOP_COUNT;
        }
        
        if (pace != "") {
            targetPace = pace.toFloat();
        } else {
            targetPace = DEFAULT_TARGET_PACE;
        }
        
        if (distance != "") {
            targetDistance = distance.toFloat();
        } else {
            targetDistance = DEFAULT_TARGET_DISTANCE;
        }
        
        destinationReached = false;
        autonomousMode = true;
        followingWaypoints = (waypointCount > 0);
        lastAvoidanceMessage = "";
        
        server.send(200, "text/plain", "Navigation started");
    });


    server.on("/stopNavigation", HTTP_GET, []() {
        autonomousMode = false;
        escServo.write(ESC_NEUTRAL);
        steeringServo.write(STEERING_CENTER);
        server.send(200, "text/plain", "Navigation stopped");
    });


    server.on("/fusionStatus", HTTP_GET, []() {
        //Serial.println("Fusion status requested");
        myGPS.setNavigationFrequency(1); // Can't seem to get fusion status at a higher frequency, don't know why
        String fusionStatus = "Failed to get fusion data";
        
        if (!myGPS.getEsfInfo()) {
            while(1) { // CHANGE THIS TO TIME OUT IN CASE IT NEVER HAPPENS!!!!!!!!!!!!!!!!!!1
                if (myGPS.getEsfInfo()) {
                    uint8_t fusionMode = myGPS.packetUBXESFSTATUS->data.fusionMode;
                    //Serial.print("Fusion Mode: ");
                    //Serial.println(fusionMode);
                
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
        float currentLat = myGPS.getLatitude() / 10000000.0;
        float currentLon = myGPS.getLongitude() / 10000000.0;
        
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
        unsigned long elapsedTime = millis() - totalTimeMs;
        
        String json = "{\"totalDistance\":" + String(totalDistance, 1) +
                      ",\"currentPace\":" + String(currentPace, 2) +
                      ",\"totalTime\":" + String(elapsedTime) +
                      ",\"currentLoop\":" + String(waypointLoopCount) +
                      ",\"targetLoops\":" + String(targetLoopCount) + "}";
                      
        server.send(200, "application/json", json);
    });

}

#endif // WEBHANDLERS_H