/*
 * Autonomous car control with GNSS navigation and obstacle avoidance.

 * Broke out main file using modular structure with separate header files.
 * Eliminated all mapping to 0-255 for speed. Speed is set only by servo style angles.
  ** where 0=max reverse, 90=neutral, 180=max forward.
 * Obstacle detection and avoidance is not currently working well, I may have broken it.
 * Pace tracking and control implemented but very poorly. Improvements needed:
  ** Fix distance tracking so it does NOT increment while no power is applied to motor
    ** (currently distance increments from GNSS noise alone even while car is still).
  ** Fix time tracking so it increments once start navigation is selected
    ** and pauses when stop navigation is selected.
  ** Add functionality to reset distance and time tracking (button probably).
  ** Investigate general improvements for pace tracking and correction. It is 
    ** VERY rough and inaccurate right now.
  ** Add functionality to show average pace using reported distance and time.
  ** Average pace is more important than instantaneous pace, though both need improvement.
 * Last updated: 2/28/2025
 */

// Include all headers
#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>

// Custom module headers
#include "config.h"
#include "sonar.h"
#include "avoidance.h"
#include "navigation.h"
#include "webhandlers.h"
#include "webpage.h"

// Define global objects
WebServer server(80);
SFE_UBLOX_GNSS myGPS;
Servo steeringServo;
Servo escServo;

// GPS and waypoint management
float targetLat = 0.0;
float targetLon = 0.0;
float waypointLats[MAX_WAYPOINTS] = {0};
float waypointLons[MAX_WAYPOINTS] = {0};
int waypointCount = 0;
int currentWaypointIndex = 0;
bool followingWaypoints = false;
bool autonomousMode = false;
bool destinationReached = false;

// Sonar readings and filtering
float frontReadings[FILTER_SAMPLES] = {0};
float leftReadings[FILTER_SAMPLES] = {0};
float rightReadings[FILTER_SAMPLES] = {0};
int readIndex = 0;
float lastFrontDist = 0;
float lastLeftDist = 0;
float lastRightDist = 0;
int currentSonar = 0;

// Status messaging
String lastAvoidanceMessage = "";

// Timing variables
unsigned long lastUpdateTime = 0;
unsigned long lastSonarUpdate = 0;
unsigned long lastAvoidanceTime = 0;
unsigned long destinationReachedTime = 0;

// Waypoint looping variables
//int waypointLoopCount = 0;        // Current count of completed loops
//int targetLoopCount = DEFAULT_LOOP_COUNT; // Target number of loops to complete

// Pace and distance tracking
float targetPace = DEFAULT_TARGET_PACE;        // Target pace in m/s
float targetDistance = DEFAULT_TARGET_DISTANCE; // Target total distance in meters
float totalDistance = 0.0;        // Total distance traveled so far
unsigned long totalTimeMs = 0;    // Total time elapsed in milliseconds
float currentPace = 0.0;          // Current pace in m/s
unsigned long lastPaceUpdate = 0; // Last time pace was calculated
float lastSegmentDistance = 0.0;  // Distance of last waypoint segment
float lastTrackedLat = 0;
float lastTrackedLon = 0;
unsigned long lastDistanceUpdate = 0;

// The HTML content for the web interface is in webpage.h

void setup() {
    unsigned long setupTime = millis();
    Serial.begin(115200);
    
    // Initialize sonar sensors
    initializeSonar();
    
    // Initialize Servo library
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    
    // Configure servo objects
    steeringServo.setPeriodHertz(50);
    escServo.setPeriodHertz(50);
    
    steeringServo.attach(STEERING_PIN, 1000, 2000); //1000ms pulse is 0 degrees, 2000ms pulse is 180 degrees
    escServo.attach(ESC_PIN, 1000, 2000); //100ms pulse is 0 degrees=max reverse, 2000ms pulse is 180 degrees=max forward
    
    // Initialize ESC - start in neutral
    escServo.write(ESC_NEUTRAL);
    delay(3000);  // Give ESC time to initialize
    
    // Center steering
    steeringServo.write(STEERING_CENTER);
    
    // Initialize GPS
    if (!initializeGPS()) {
        Serial.println("GPS initialization failed!");
    }
    
    // Setup WiFi Access Point
    WiFi.softAP(ssid, password);

    // Setup web server routes
    setupWebServerRoutes();
    server.begin();
}

void loop() {
    // Always handle web server requests and update sensors
    server.handleClient();
    updateSonarReadings();
    
    // Check if we need to clear avoidance or destination messages
    updateStatusMessages();
    
    // Safety timeout check - applies to both manual and autonomous modes
    if (millis() - lastUpdateTime > TIMEOUT_MS) {
        escServo.write(ESC_NEUTRAL);
        steeringServo.write(STEERING_CENTER);
    }
    
    // Autonomous mode with valid GPS fix
    if (autonomousMode && myGPS.getFixType() > 0) {
        float currentLat, currentLon;
        getCurrentPosition(currentLat, currentLon);
        
        // Track continuous distance
        updateContinuousDistance(currentLat, currentLon);
        
        // Calculate distance to target and steering angle
        float distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
        int steeringAngle = calculateSteeringAngle(currentLat, currentLon);
        
        // Check for obstacles first
        if (!checkObstacles(steeringAngle)) {
            // No obstacles, check if waypoint reached
            if (distance < WAYPOINT_REACHED_RADIUS) {
                handleWaypointReached();
            }
        }
        
        // Get updated steering angle from avoidance system
        steeringAngle = handleAvoidance(steeringAngle);
        
        // Update pace and speed settings
        updatePaceControl();
        
        // Apply final steering angle with constraints
        steeringAngle = constrain(steeringAngle, 
                              STEERING_CENTER - STEERING_MAX, 
                              STEERING_CENTER + STEERING_MAX);
        steeringServo.write(steeringAngle);
    }
    
    delay(2);  // Small delay for stability
}

// Track continuous distance traveled
void updateContinuousDistance(float currentLat, float currentLon) {
    if (millis() - lastDistanceUpdate >= 1000) { // Update once per second
        if (lastTrackedLat != 0 && lastTrackedLon != 0) {
            float movementDistance = calculateDistance(lastTrackedLat, lastTrackedLon, currentLat, currentLon);
            
            // Only add reasonable distances to prevent GPS jitter
            if (movementDistance > 0.1 && movementDistance < 5.0) {
                totalDistance += movementDistance;
                //Serial.print("Added distance: ");
                //Serial.print(movementDistance);
                //Serial.print(" Total: ");
                //Serial.println(totalDistance);
                
                // Check distance target if set
                if (targetDistance > 0 && totalDistance >= targetDistance) {
                    autonomousMode = false;
                    destinationReached = true;
                    destinationReachedTime = millis();
                    lastAvoidanceMessage = "Target distance reached";
                    escServo.write(ESC_NEUTRAL); // Stop
                    return; // Exit early as we're no longer in autonomous mode
                }
            }
        }
        
        // Update last tracked position
        lastTrackedLat = currentLat;
        lastTrackedLon = currentLon;
        lastDistanceUpdate = millis();
    }
}

// Update message timeouts
void updateStatusMessages() {
    if (lastAvoidanceMessage != "") {
        if (lastAvoidanceMessage == "Destination reached" || 
            lastAvoidanceMessage == "Target distance reached") {
            // Clear destination message after timeout
            if (millis() - destinationReachedTime > DESTINATION_MESSAGE_TIMEOUT) {
                lastAvoidanceMessage = "";
                destinationReached = false;  // Reset for next navigation
            }
        } else {
            // Handle other avoidance messages
            if (millis() - lastAvoidanceTime > AVOIDANCE_MESSAGE_TIMEOUT) {
                lastAvoidanceMessage = "";
            }
        }
    }
}

// Handle reaching a waypoint
void handleWaypointReached() {
    // Update total distance - add the segment that was just completed
    if (followingWaypoints) {
        totalDistance += lastSegmentDistance;
    }
    
    // Following waypoints (not just going to a single destination)
    if (followingWaypoints) {
        // Move to next waypoint
        if (currentWaypointIndex < waypointCount - 1) {
            // Go to next waypoint in current sequence
            currentWaypointIndex++;
        } 
        else {
            // End of waypoint list reached - wrap around
            currentWaypointIndex = 0;
            lastAvoidanceMessage = "Starting waypoint sequence again";
        }
        
        // Set next target waypoint
        targetLat = waypointLats[currentWaypointIndex];
        targetLon = waypointLons[currentWaypointIndex];
        
        // Calculate and store the distance to the next waypoint
        float currentLat, currentLon;
        getCurrentPosition(currentLat, currentLon);
        lastSegmentDistance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
    } 
    else {
        // Single destination reached
        autonomousMode = false;
        destinationReached = true;
        destinationReachedTime = millis();
        lastAvoidanceMessage = "Destination reached";
        escServo.write(ESC_NEUTRAL); // Stop
    }
}

