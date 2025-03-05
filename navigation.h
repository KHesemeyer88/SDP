#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "config.h" // For constants like WAYPOINT_REACHED_RADIUS

// External variables
extern SFE_UBLOX_GNSS myGPS;
extern float targetLat, targetLon;
extern float waypointLats[], waypointLons[];
extern int waypointCount, currentWaypointIndex;
extern bool followingWaypoints;
extern unsigned long lastUpdateTime;

// Pace control variables and tracking variables
extern bool autonomousMode;
extern unsigned long lastPaceUpdate;
extern float targetPace;
extern float currentPace;
extern float totalDistance;
extern float averagePace;
extern float lastTrackedLat;
extern float lastTrackedLon;
extern unsigned long lastDistanceUpdate;

extern unsigned long startTime;         // Set when autonomous mode starts
extern unsigned long finalElapsedTime;    // Set when autonomous mode stops

extern String lastAvoidanceMessage;

extern Servo escServo;

// Initialize GPS module
bool initializeGPS() {
    Wire.begin(32, 33);
    if (!myGPS.begin()) {
        Serial.println("u-blox GNSS not detected. Check wiring.");
        return false;
    }
    myGPS.setI2COutput(COM_TYPE_UBX);
    myGPS.setNavigationFrequency(10);
    return true;
}

// Get fusion status from IMU 
String getFusionStatus() {
    // Temporarily set nav frequency to 1Hz for status check
    myGPS.setNavigationFrequency(1);
    String fusionStatus = "Failed to get fusion data";

    // Try to get ESF info with timeout
    unsigned long startTimeLocal = millis();
    const unsigned long ESF_TIMEOUT = 1000; // 1 second timeout

    if (!myGPS.getEsfInfo()) {
        while (millis() - startTimeLocal < ESF_TIMEOUT) {
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

    // Restore original navigation frequency
    myGPS.setNavigationFrequency(10);
    return fusionStatus;
}

// Calculate distance between two coordinates using the Haversine formula
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    float lat1Rad = lat1 * PI / 180.0;
    float lon1Rad = lon1 * PI / 180.0;
    float lat2Rad = lat2 * PI / 180.0;
    float lon2Rad = lon2 * PI / 180.0;

    float dLat = lat2Rad - lat1Rad;
    float dLon = lon2Rad - lon1Rad;
    float a = sin(dLat/2) * sin(dLat/2) +
              cos(lat1Rad) * cos(lat2Rad) *
              sin(dLon/2) * sin(dLon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    return c * 6371000; // Earth radius in meters
}

// Calculate bearing between two coordinates
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    float lat1Rad = lat1 * PI / 180.0;
    float lon1Rad = lon1 * PI / 180.0;
    float lat2Rad = lat2 * PI / 180.0;
    float lon2Rad = lon2 * PI / 180.0;

    float dLon = lon2Rad - lon1Rad;
    float y = sin(dLon) * cos(lat2Rad);
    float x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(dLon);


    float bearing = atan2(y, x) * 180.0 / PI;
    return fmod((bearing + 360.0), 360.0);
}

// Get current GPS coordinates
void getCurrentPosition(float &lat, float &lon) {
    lat = myGPS.getLatitude() / 10000000.0;
    lon = myGPS.getLongitude() / 10000000.0;
}

// Record a waypoint at the current position
bool recordWaypoint() {
    if (waypointCount >= MAX_WAYPOINTS)
        return false; // Waypoint limit reached


    float lat, lon;
    getCurrentPosition(lat, lon);

    waypointLats[waypointCount] = lat;
    waypointLons[waypointCount] = lon;
    waypointCount++;

    return true;
}

// Clear all waypoints
void clearWaypoints() {
    waypointCount = 0;
}

// Calculate steering angle based on heading error (simple proportional control)
int calculateSteeringAngle(float currentLat, float currentLon) {
    if (myGPS.getFixType() == 0)
        return STEERING_CENTER;

    float distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
    float bearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
    float currentHeading = myGPS.getHeading() / 100000.0;
    
    float headingError = bearing - currentHeading;
    if (headingError > 180)
        headingError -= 360;
    
    if (headingError < -180)
        headingError += 360;
    
    int steeringAngle = STEERING_CENTER + (headingError * 0.25);
    // Update timestamp
    lastUpdateTime = millis();

    return steeringAngle;
}

// pace control and compute average pace
void updatePaceControl() {
    if (autonomousMode) {
        unsigned long currentTime = millis();
        if (currentTime - lastPaceUpdate > SPEED_CORRECTION_INTERVAL) {
            float speedMps = myGPS.getGroundSpeed() / 1000.0;
            currentPace = speedMps;
            
            unsigned long elapsedTime = currentTime - startTime;
            if (elapsedTime > 0) {
                averagePace = totalDistance / (elapsedTime / 1000.0);
            }
            
            if (targetPace > 0 && lastAvoidanceMessage == "") {
                int currentEscValue = escServo.read();
                float paceRatio = targetPace > 0 ? currentPace / targetPace : 1.0;
                int adjustmentValue = (abs(1.0 - paceRatio) < 0.05) ? (int)((1.0 - paceRatio) * 5.0)
                                                                   : (int)((1.0 - paceRatio) * 15.0);
                int newEscValue = currentEscValue + adjustmentValue;
                newEscValue = constrain(newEscValue, 105, 135);
                escServo.write(newEscValue);
            }
            lastPaceUpdate = currentTime;
        }
    }
}

// Update distance tracking: add incremental distance only when motor power is applied
void updateDistanceTracking() {
    float currentLat, currentLon;
    getCurrentPosition(currentLat, currentLon);
    if (millis() - lastDistanceUpdate >= 1000 && escServo.read() != ESC_NEUTRAL) {
        if (lastTrackedLat != 0 && lastTrackedLon != 0) {
            float delta = calculateDistance(lastTrackedLat, lastTrackedLon, currentLat, currentLon);
            if (delta < 5.0)
                totalDistance += delta;
            if (targetDistance > 0 && totalDistance >= targetDistance) {
                autonomousMode = false;
                destinationReached = true;
                finalElapsedTime = millis() - startTime;  // Capture final elapsed time
                lastAvoidanceMessage = "Target distance reached";
                escServo.write(ESC_NEUTRAL);
                return;
            }
        }
        lastTrackedLat = currentLat;
        lastTrackedLon = currentLon;
        lastDistanceUpdate = millis();
    }
}

// Reset distance and time tracking for a new session
void resetTracking() {
    totalDistance = 0.0;
    startTime = (autonomousMode) ? millis() : 0;
    float currentLat, currentLon;
    getCurrentPosition(currentLat, currentLon);
    lastTrackedLat = currentLat;
    lastTrackedLon = currentLon;
}

// update obstacle avoidance and destination reached flash messages
void updateStatusMessages() {
  if (lastAvoidanceMessage != "") {
    if (lastAvoidanceMessage == "Destination reached" ||
        lastAvoidanceMessage == "Target distance reached") {
      if (millis() - destinationReachedTime > DESTINATION_MESSAGE_TIMEOUT) {
        lastAvoidanceMessage = "";
        destinationReached = false;
      }
    } else {
      if (millis() - lastAvoidanceTime > AVOIDANCE_MESSAGE_TIMEOUT) {
        lastAvoidanceMessage = "";
      }
    }
  }
}

// increment waypoint index, calculate segment distance, stop auto nav if distanced reached
void handleWaypointReached() {
  if (followingWaypoints) {
    if (currentWaypointIndex < waypointCount - 1) {
      currentWaypointIndex++;
    } else {
      currentWaypointIndex = 0;
      lastAvoidanceMessage = "Starting waypoint sequence again";
    }
    targetLat = waypointLats[currentWaypointIndex];
    targetLon = waypointLons[currentWaypointIndex];
    float currentLat, currentLon;
    getCurrentPosition(currentLat, currentLon);
    lastSegmentDistance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
  } else {
    autonomousMode = false;
    destinationReached = true;
    destinationReachedTime = millis();
    finalElapsedTime = millis() - startTime;  // Capture the final elapsed time
    lastAvoidanceMessage = "Destination reached";
    escServo.write(ESC_NEUTRAL);
  }
}

#endif // NAVIGATION_H