// navigation.h
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
// Additional extern declarations for pace control
extern bool autonomousMode;
extern unsigned long lastPaceUpdate;
extern float targetPace;
extern float currentPace;
extern unsigned long totalTimeMs;
extern AvoidanceState getAvoidanceState();
extern Servo escServo;

// Initialize GPS module
bool initializeGPS() {
    Wire.begin(32, 33);
    if (myGPS.begin() == false) {
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
    unsigned long startTime = millis();
    const unsigned long ESF_TIMEOUT = 1000; // 1 second timeout
    
    if (!myGPS.getEsfInfo()) {
        while(millis() - startTime < ESF_TIMEOUT) {
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

// Calculate distance between two coordinates using Haversine formula
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
    float x = cos(lat1Rad) * sin(lat2Rad) -
              sin(lat1Rad) * cos(lat2Rad) * cos(dLon);
    
    float bearing = atan2(y, x) * 180.0 / PI;
    return fmod((bearing + 360.0), 360.0);
}

// Get current GPS coordinates
void getCurrentPosition(float &lat, float &lon) {
    lat = myGPS.getLatitude() / 10000000.0;
    lon = myGPS.getLongitude() / 10000000.0;
}

// Record a waypoint at current position
bool recordWaypoint() {
    if (waypointCount >= MAX_WAYPOINTS) {
        return false; // Waypoint limit reached
    }
    
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

// Calculate steering angle based on heading error
int calculateSteeringAngle(float currentLat, float currentLon) {
    if (myGPS.getFixType() == 0) {
        return STEERING_CENTER; // No GPS fix, return center
    }
    
    float distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
    float bearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
    float currentHeading = myGPS.getHeading() / 100000.0;

    float headingError = bearing - currentHeading;
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;

    // Base steering calculation with PID-like proportional control
    int steeringAngle = STEERING_CENTER + (headingError * 0.25);
    
    // Update timestamp
    lastUpdateTime = millis();
    
    return steeringAngle;
}

void updatePaceControl() {
    if (autonomousMode) {
        totalTimeMs = millis();
        
        // Update pace calculation every interval
        if (millis() - lastPaceUpdate > SPEED_CORRECTION_INTERVAL) {
            // Calculate current speed from GPS (m/s)
            float speedMps = myGPS.getGroundSpeed() / 1000.0; // Convert from mm/s to m/s
            
            // Update current pace
            currentPace = speedMps;
            
            // Adjust speed if pace control is active
            if (targetPace > 0 && getAvoidanceState() == NO_OBSTACLE) {
                // Get current ESC value
                int currentEscValue = escServo.read();
                
                // Calculate pace difference as a percentage of target
                float paceRatio = targetPace > 0 ? currentPace / targetPace : 1.0;
                
                // More aggressive adjustment factor
                int adjustmentValue;
                
                if (paceRatio < 0.95) {
                    // Too slow - speed up more aggressively
                    adjustmentValue = (int)((1.0 - paceRatio) * 15.0);
                } else if (paceRatio > 1.05) {
                    // Too fast - slow down more aggressively
                    adjustmentValue = (int)((1.0 - paceRatio) * 15.0);
                } else {
                    // Close enough - minor adjustment
                    adjustmentValue = (int)((1.0 - paceRatio) * 5.0);
                }
                
                // Calculate new ESC value
                int newEscValue = currentEscValue + adjustmentValue;
                newEscValue = constrain(newEscValue, 105, 135); // Use safe range
                
                // Set the ESC directly
                escServo.write(newEscValue);
            }
            
            lastPaceUpdate = millis();
        }
    }
}
#endif // NAVIGATION_H