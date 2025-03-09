#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "config.h" // For constants like WAYPOINT_REACHED_RADIUS

// External GNSS variables
extern SFE_UBLOX_GNSS myGPS;
extern float targetLat, targetLon;
extern float waypointLats[], waypointLons[];
extern int waypointCount, currentWaypointIndex;
extern bool followingWaypoints;

// External pace control and tracking variables
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

extern Servo escServo;

// store the latest GPS data
extern volatile float currentLat, currentLon;
extern volatile float currentSpeed;
extern volatile uint8_t currentFixType;
extern volatile bool newPVTDataAvailable;

// Add this to the top of navigation.h
void pvtCallback(UBX_NAV_PVT_data_t *pvtData);

// Update the callback function implementation
void pvtCallback(UBX_NAV_PVT_data_t *pvtData)
{
    //Serial.println("PVT Callback called!");
    
    // Update global position variables with new data
    currentLat = pvtData->lat / 10000000.0;
    currentLon = pvtData->lon / 10000000.0;
    currentSpeed = pvtData->gSpeed / 1000.0;  // Convert from mm/s to m/s
    currentFixType = pvtData->fixType;
    
    // Set flag to indicate new data is available
    newPVTDataAvailable = true;
}

// Initialize GPS module
bool initializeGPS() {
    Wire.begin(32, 33);
    if (!myGPS.begin()) {
        Serial.println("u-blox GNSS not detected. Check wiring.");
        return false;
    }
    
    Serial.println("GNSS module found!");
    
    myGPS.setI2COutput(COM_TYPE_UBX);
    myGPS.setNavigationFrequency(NAV_FREQ);
    
    // Enable the callback for PVT messages
    myGPS.setAutoPVTcallbackPtr(&pvtCallback);
    
    // Set Dynamic Model to Robotic Lawn Mower (11) 
    // Also need to try: portable (0), pedestrian (3), automotive (4), wrist (9), bike(10), escooter (12)
    // if (!myGPS.setVal8(0x20110021, 11)) {
    //     Serial.println("Failed to set dynamic model to RLM.");
    // } else {
    //     Serial.println("Dynamic model set to Robotic Lawn Mower (RLM).");
    // }
    // Disable all sensor fusion: (does not seem to improve performance)
    if (!myGPS.setVal8(UBLOX_CFG_SFCORE_USE_SF, 0)) {
        Serial.println("Failed to disable sensor fusion.");
    } else {
        Serial.println("IMU sensor fusion disabled.");
    }
    
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
    myGPS.setNavigationFrequency(NAV_FREQ);
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
    lat = currentLat;
    lon = currentLon;
}

// Record a waypoint at the current position
bool recordWaypoint() {
    if (waypointCount >= MAX_WAYPOINTS)
        return false; // Waypoint limit reached

    waypointLats[waypointCount] = currentLat;
    waypointLons[waypointCount] = currentLon;
    waypointCount++;

    return true;
}

// Clear all waypoints
void clearWaypoints() {
    waypointCount = 0;
}

// Calculate steering angle based on heading error (simple proportional control)
int calculateSteeringAngle(float currentLat, float currentLon) {
    if (currentFixType == 0)
        return STEERING_CENTER;

    float distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
    float bearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
    float currentHeading = myGPS.getHeading() / 100000.0;  // Still need to poll for heading
    
    float headingError = bearing - currentHeading;
    if (headingError > 180)
        headingError -= 360;
    
    if (headingError < -180)
        headingError += 360;
    
    int steeringAngle = STEERING_CENTER + (headingError * 0.25);

    return steeringAngle;
}

// pace control and compute average pace
// Global variable for speed smoothing
const int SPEED_SAMPLES = 3;
volatile float speedSamples[SPEED_SAMPLES] = {0};
volatile int speedSampleIndex = 0;

// pace control and compute average pace
void updatePaceControl() {
    if (autonomousMode) {
        unsigned long currentTime = millis();
        if (currentTime - lastPaceUpdate > SPEED_CORRECTION_INTERVAL) {
            // Update speed sample buffer for smoothing
            speedSamples[speedSampleIndex] = currentSpeed;
            speedSampleIndex = (speedSampleIndex + 1) % SPEED_SAMPLES;
            
            // Calculate smoothed speed (simple moving average)
            float smoothedSpeed = 0;
            for (int i = 0; i < SPEED_SAMPLES; i++) {
                smoothedSpeed += speedSamples[i];
            }
            smoothedSpeed /= SPEED_SAMPLES;
            
            // Use smoothed speed for pace control
            currentPace = smoothedSpeed;
            
            unsigned long elapsedTime = currentTime - startTime;
            if (elapsedTime > 0) {
                averagePace = totalDistance / (elapsedTime / 1000.0);
            }
            
            // Only apply throttle control when we have a target pace and not avoiding obstacles
            if (targetPace > 0 && lastAvoidanceMessage == "") {
                // Calculate speed error (difference between target and current)
                float speedError = targetPace - currentPace;
                
                // PI control variables
                static float integralError = 0;
                float dt = SPEED_CORRECTION_INTERVAL / 1000.0; // Time step in seconds
                
                // If we're at a standstill, provide an initial push
                if (currentPace < 0.2 && targetPace > 0) {
                    // Initial push to get moving
                    escServo.write(ESC_MIN_FWD + 12);
                    // Reset integral to prevent windup during startup
                    integralError = 0;
                } else {
                    // Update integral term (accumulate error over time)
                    integralError += speedError * dt;
                    
                    // Anti-windup: Limit the integral term to prevent excessive accumulation
                    integralError = constrain(integralError, -8.0, 8.0);
                    
                    // PI control equation: Output = Kp*error + Ki*integral
                    float Kp = 20.0;  // Proportional gain
                    float Ki = 10.0;   // Integral gain
                    
                    int pidOutput = (int)(Kp * speedError + Ki * integralError);
                    
                    // Calculate ESC value: base value + PI controller output
                    int escValue = ESC_MIN_FWD + pidOutput;
                    
                    // Constrain to valid range
                    escValue = constrain(escValue, ESC_MIN_FWD, ESC_MAX_FWD);
                    
                    // apply the throttle setting
                    escServo.write(escValue);
                }
            } else if (targetPace <= 0 || lastAvoidanceMessage != "") {
                // Reset integral term when we don't want to move
                static float integralError = 0;
                integralError = 0;
            }
            
            lastPaceUpdate = currentTime;
        }
    }
}

// Update distance tracking: add incremental distance only when motor power is applied
// Update distance tracking: add incremental distance only when GPS reports movement
void updateDistanceTracking() {
    if (millis() - lastDistanceUpdate >= 1000) {
        // Only update if we have valid previous coordinates
        if (lastTrackedLat != 0 && lastTrackedLon != 0 && currentLat != 0 && currentLon != 0) {
            float delta = calculateDistance(lastTrackedLat, lastTrackedLon, currentLat, currentLon);
            
            // Use a looser filter since we're checking speed as well
            if (delta < 10.0 && currentSpeed > 0.5) {
                totalDistance += delta;
            }
            
            if (targetDistance > 0 && totalDistance >= targetDistance) {
                autonomousMode = false;
                destinationReached = true;
                finalElapsedTime = millis() - startTime;
                lastAvoidanceMessage = "Target distance reached";
                escServo.write(ESC_NEUTRAL);
                return;
            }
        }
        
        // Update tracked position regardless of whether we calculated distance
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