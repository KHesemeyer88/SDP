/*
 * RC Car Control with GPS Navigation and Obstacle Avoidance
 * Main file using modular structure with separate header files
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
int waypointLoopCount = 0;        // Current count of completed loops
int targetLoopCount = DEFAULT_LOOP_COUNT; // Target number of loops to complete

// Pace and distance tracking
float targetPace = DEFAULT_TARGET_PACE;        // Target pace in m/s
float targetDistance = DEFAULT_TARGET_DISTANCE; // Target total distance in meters
float totalDistance = 0.0;        // Total distance traveled so far
unsigned long totalTimeMs = 0;    // Total time elapsed in milliseconds
float currentPace = 0.0;          // Current pace in m/s
unsigned long lastPaceUpdate = 0; // Last time pace was calculated
float lastSegmentDistance = 0.0;  // Distance of last waypoint segment

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
    
    steeringServo.attach(STEERING_PIN, 1000, 2000);
    escServo.attach(ESC_PIN, 1000, 2000);
    
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
    // Prevent movement if navigation is stopped
    if (!autonomousMode) {
        escServo.write(ESC_NEUTRAL);
        steeringServo.write(STEERING_CENTER);
        server.handleClient(); // Keep handling web requests
        delay(2);  // Prevent CPU overload
        return;  // Skip the rest of the movement logic
    }

    // Update sonar readings
    updateSonarReadings();

    // Autonomous navigation with obstacle avoidance
    if (autonomousMode && myGPS.getFixType() > 0) {
        float currentLat, currentLon;
        getCurrentPosition(currentLat, currentLon);
        
        // Calculate steering angle based on current position and target
        int steeringAngle = calculateSteeringAngle(currentLat, currentLon);
        
        float distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
            
        // Check for obstacles
        if (!checkObstacles(steeringAngle)) {
            // No obstacles, check if destination reached
            if (distance < WAYPOINT_REACHED_RADIUS) {
                // Update total distance - add the segment that was just completed
                if (followingWaypoints) {
                    totalDistance += lastSegmentDistance;
                }
                
                // Check if we've hit distance target (if set)
                if (targetDistance > 0 && totalDistance >= targetDistance) {
                    // Target distance reached
                    autonomousMode = false;
                    destinationReached = true;
                    destinationReachedTime = millis();
                    lastAvoidanceMessage = "Target distance reached";
                    escServo.write(ESC_NEUTRAL); // Stop
                }
                // Following waypoints (not just going to a single destination)
                else if (followingWaypoints) {
                    // Move to next waypoint
                    if (currentWaypointIndex < waypointCount - 1) {
                        // Go to next waypoint in current sequence
                        currentWaypointIndex++;
                    } 
                    else {
                        // End of waypoint list reached - wrap around instead of counting loops
                        currentWaypointIndex = 0;
                        lastAvoidanceMessage = "Starting waypoint sequence again";
                        
                        // Only stop if we have a distance target, otherwise keep going
                        if (targetDistance > 0 && totalDistance >= targetDistance) {
                            autonomousMode = false;
                            destinationReached = true;
                            destinationReachedTime = millis();
                            lastAvoidanceMessage = "Target distance reached";
                            escServo.write(ESC_NEUTRAL); // Stop
                        }
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
        }
        
        // Get updated steering angle from avoidance system
        steeringAngle = handleAvoidance(steeringAngle);

        // Update time tracking and pace if we're in autonomous mode
        if (autonomousMode && myGPS.getFixType() > 0) {
            totalTimeMs = millis();
            // Update pace calculation every second
            if (millis() - lastPaceUpdate > SPEED_CORRECTION_INTERVAL) {
                // Calculate current speed from GPS (m/s)
                float speedMps = myGPS.getGroundSpeed() / 1000.0; // Convert from mm/s to m/s
                
                // Update current pace
                currentPace = speedMps;
                
                // Adjust speed if pace control is active
                if (targetPace > 0) {
                    // Get current throttle setting - assuming a global variable for this
                    int currentSpeed = map(escServo.read(), ESC_MIN_FWD, ESC_MAX_FWD, 0, 255);
        
                    // Adjust speed based on pace difference
                    float paceDiff = targetPace - currentPace;
                    int speedAdjustment = (int)(paceDiff * SPEED_CORRECTION_FACTOR * 255);
                    
                    // Apply new speed with constraints
                    currentSpeed = constrain(currentSpeed + speedAdjustment, 0, 255);
                    
                    // Only apply this in NO_OBSTACLE state, otherwise obstacle avoidance handles speed
                    if (getAvoidanceState() == NO_OBSTACLE) {
                        escServo.write(map(currentSpeed, 0, 255, ESC_MIN_FWD, ESC_MAX_FWD));
                    }
                }
                
                lastPaceUpdate = millis();
            }
        }        
        // Apply final steering angle with constraints
        steeringAngle = constrain(steeringAngle, 
                              STEERING_CENTER - STEERING_MAX, 
                              STEERING_CENTER + STEERING_MAX);
        steeringServo.write(steeringAngle);
    }

    // Clear messages
    if (lastAvoidanceMessage != "") {
        if (lastAvoidanceMessage == "Destination reached") {
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

    // Handle web server requests
    server.handleClient();

    // Safety timeout - stop if no updates received
    if (millis() - lastUpdateTime > TIMEOUT_MS) {
        escServo.write(ESC_NEUTRAL);
        steeringServo.write(STEERING_CENTER);
    }

    delay(2);  // Small delay for stability
}