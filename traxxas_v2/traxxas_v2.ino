/*
 * Autonomous car control with GNSS navigation and obstacle avoidance.

 * Broke out main file using modular structure with separate header files.

 * Obstacle detection and avoidance is not currently working well.

  ** needs further testing on - 
  ** average pace and instantaneous pace, both need improvement.
  ** general improvements for pace tracking and correction. 

 * Last updated: 3/5/2025
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>

#include "config.h"
#include "sonar.h"
#include "avoidance.h"  
#include "navigation.h"
#include "webhandlers.h"
#include "webpage.h"

// Global objects
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
unsigned long lastCommandTime = 0;
unsigned long lastSonarUpdate = 0;
unsigned long lastAvoidanceTime = 0;
unsigned long destinationReachedTime = 0;

// Pace and distance tracking variables
float targetPace = DEFAULT_TARGET_PACE;
float targetDistance = DEFAULT_TARGET_DISTANCE;
float totalDistance = 0.0;
unsigned long startTime = 0;         // Record when autonomous mode begins
unsigned long finalElapsedTime = 0;    // Capture elapsed time when mode stops

float currentPace = 0.0;
unsigned long lastPaceUpdate = 0;
float lastSegmentDistance = 0.0;
float lastTrackedLat = 0;
float lastTrackedLon = 0;
unsigned long lastDistanceUpdate = 0;
float averagePace = 0.0;

// GPS data from PVT callback
volatile float currentLat, currentLon;
volatile float currentSpeed;
volatile uint8_t currentFixType;
volatile bool newPVTDataAvailable;

// Initial straight-line phase variables
bool initialStraightPhase = false;
unsigned long straightPhaseStartTime = 0;
const unsigned long STRAIGHT_PHASE_DURATION = 500; // initial straight phase

void setup() {
  Serial.begin(115200);
  
  // Initialize sonar sensors
  initializeSonar();
  
  // Allocate timers for the servo library
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Configure servo objects
  steeringServo.setPeriodHertz(50);
  escServo.setPeriodHertz(50);
  
  steeringServo.attach(STEERING_PIN, 1000, 2000);
  escServo.attach(ESC_PIN, 1000, 2000);
  
  // Initialize ESC to neutral and allow time to initialize
  escServo.write(ESC_NEUTRAL);
  delay(3000);
  
  // Center steering
  steeringServo.write(STEERING_CENTER);
  
  // Initialize GPS
  if (!initializeGPS()) {
    while(1){
        Serial.println("GPS initialization failed!");
        delay(1000);
        if(initializeGPS()){
          break;
        }
    }
  }
  
  // Setup WiFi as an access point
  WiFi.softAP(ssid, password);
  
  // Setup web server routes and start the server
  setupWebServerRoutes();
  server.begin();
}

void loop() {
  // Process incoming GNSS messages
  myGPS.checkUblox();  // Check for the arrival of new data and process it
  myGPS.checkCallbacks(); // Check if any callbacks are waiting to be processed
  
  // Safety checks run regardless of MODE
  unsigned long currentTime = millis();
  
  // Safety check: if no clients connected to the AP, stop the vehicle
  if (WiFi.softAPgetStationNum() == 0) {
      autonomousMode = false;
      escServo.write(ESC_NEUTRAL);
      steeringServo.write(STEERING_CENTER);
  }

  // Check for command staleness in manual mode
  if (!autonomousMode && (currentTime - lastCommandTime > COMMAND_TIMEOUT_MS)) {
    escServo.write(ESC_NEUTRAL);
    steeringServo.write(STEERING_CENTER);
  }

  // handle web requests
  server.handleClient();

  // ALWAYS update sonar readings
  updateSonarReadings();
  // emergency check for avoidance
     int emergencyAovidance = applyObstacleAvoidance(STEERING_CENTER);
  
  // do autonomous navigation if in auto mode:
  if (autonomousMode) { //only update status messages in auto mode
      updateStatusMessages();
      // only do navigation in auto mode with good GNSS fix
      if (currentFixType > 0) {
          // Calculate distance to target and base steering angle
          float distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
          
          // Check if we're in initial straight-line phase
          if (initialStraightPhase) {
              // During straight phase, keep steering centered
              steeringServo.write(STEERING_CENTER);
              
              // If straight phase duration has elapsed, exit straight phase
              if (currentTime - straightPhaseStartTime >= STRAIGHT_PHASE_DURATION) {
                  initialStraightPhase = false;
              }
          } else {
              // Normal navigation with steering correction
              int steeringAngle = calculateSteeringAngle(currentLat, currentLon);
              steeringAngle = applyObstacleAvoidance(steeringAngle);

              // Constrain and apply the steering angle
              steeringAngle = constrain(steeringAngle, STEERING_CENTER - STEERING_MAX, STEERING_CENTER + STEERING_MAX);
              
              if (lastAvoidanceMessage == "") {
                steeringServo.write(steeringAngle);
              }
              
              // If the target is reached, handle waypoint or end session
              if (distance < WAYPOINT_REACHED_RADIUS) {
                  handleWaypointReached();
                  // No longer reset the straight-line phase for each waypoint
              }
          }
          
          // Update pace control (do this regardless of straight phase)
          updatePaceControl();
          
          // Update distance tracking
          updateDistanceTracking();
          
          // Reset the new data flag after processing
          newPVTDataAvailable = false;
      }
  }
  delay(1);
}