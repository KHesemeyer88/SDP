/*
 This is the first pass at integrating RTK.
 Switched to client mode from AP. 
 On 3/11/25, tried field test. Car went nowhere near the WP, wild and erratic.

 To do:
 (1) implement websockets
 (2) investigate better RTK code that doesn't block while waiting for corrections.
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
#include "websocket_handler.h" 

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

volatile int carrSoln;
volatile double hAcc;

volatile CorrectionStatus rtcmCorrectionStatus = CORR_NONE;
unsigned long correctionAge = 0;

// Initial straight-line phase variables
bool initialStraightPhase = false;
unsigned long straightPhaseStartTime = 0;
const unsigned long STRAIGHT_PHASE_DURATION = 1000; // initial straight phase

// Driving state tracking
bool drivingState = false;
unsigned long lastNonNeutralCommand = 0;
const unsigned long DRIVING_GRACE_PERIOD = 1500; // 1.5 seconds grace period

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial more time to stabilize
  Serial.println("\n\nRC Car starting up...");
  
  // Initialize lidar sensors
  initializeSonar();
  delay(100);
  
  // Servo setup with delays between operations
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  delay(100);
  
  steeringServo.setPeriodHertz(50);
  escServo.setPeriodHertz(50);
  delay(100);
  
  steeringServo.attach(STEERING_PIN, 1000, 2000);
  escServo.attach(ESC_PIN, 1000, 2000);
  delay(100);
  
  // Initialize ESC to neutral and allow time to initialize
  escServo.write(ESC_NEUTRAL);
  Serial.println("Servos initialized, waiting for ESC...");
  delay(2000);
  
  // Center steering
  steeringServo.write(STEERING_CENTER);
  delay(100);
  
  // Setup WiFi first to get IP address
  Serial.printf("Connecting to WiFi network: %s\n", ssid);
  WiFi.begin(ssid, password);
  
  // Wait for connection with timeout
  int wifiTimeout = 0;
  while (WiFi.status() != WL_CONNECTED && wifiTimeout < 20) {
    delay(500);
    Serial.print(".");
    wifiTimeout++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.printf("Connected to %s\n", ssid);
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("Failed to connect to WiFi");
  }
  
  delay(500); // Extra delay before web server setup
  
  // Setup web server
  setupWebServerRoutes();
  initWebSockets();
  server.begin();
  Serial.println("Web server started");
  
  delay(500); // Extra delay before GNSS init
  
  // Initialize GNSS
  Serial.println("Initializing GPS...");
  if (!initializeGPS()) {
    Serial.println("GNSS initialization failed. Will retry in loop");
    while(1) {
      server.handleClient(); // Process web requests while waiting
      Serial.println("Retrying GNSS initialization...");
      delay(1000);
      if (initializeGPS()){
        Serial.println("GNSS initialized successfully");
        break;
      }
    }
  } else {
    Serial.println("GNSS initialized successfully");
  }
  
  Serial.println("Setup complete. Entering main loop.");
}
void loop() {
  // SECTION 1: Things that ALWAYS happen regardless of state
  webSocket.loop();
  server.handleClient();
  updateWebSocketClients();
  unsigned long currentTime = millis();
  
  // Update driving state
  if (autonomousMode) {
    drivingState = false;  // Force non-driving state in autonomous mode
  } else if (drivingState && (currentTime - lastNonNeutralCommand > DRIVING_GRACE_PERIOD)) {
    drivingState = false;  // Exit driving state after grace period
  }
  
  // Safety timeouts
  if (!autonomousMode && (currentTime - lastCommandTime > COMMAND_TIMEOUT_MS)) {
    escServo.write(ESC_NEUTRAL);
    steeringServo.write(STEERING_CENTER);
  }
  
  // Handle WiFi reconnection if needed (without Serial prints)
  while (WiFi.status() != WL_CONNECTED) {
    // Stop the car for safety during disconnection
    escServo.write(ESC_NEUTRAL);
    steeringServo.write(STEERING_CENTER);
    server.handleClient();
    webSocket.loop();
    updateWebSocketClients();
    delay(100);
  }
  
  // SECTION 2: State-specific processing (drivingState means we are actively driving in manual mode)
  if (!drivingState) {
    // NON-DRIVING STATE - Full processing for idle or autonomous mode
    
    // Always process in non-driving state
    myGPS.checkUblox();
    myGPS.checkCallbacks();
    updateSonarReadings();
    
    // RTK connection handling
    if (!ntripClient.connected()) {
      connectToNTRIP();
    } else {
      processConnection();
    }
    
    // Reconnect to NTRIP if needed and connected to WiFi
    if (!ntripClient.connected() && WiFi.status() == WL_CONNECTED) {
      static unsigned long lastReconnectAttempt = 0;
      if (currentTime - lastReconnectAttempt > 1000) { // Try every 1 second
        lastReconnectAttempt = currentTime;
        connectToNTRIP();
      }
    }
    
    // Autonomous mode processing
    if (autonomousMode && currentFixType > 0) {
      // Process status messages
      updateStatusMessages();
      
      // Calculate distance to target
      float distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
      
      // Handle initial straight-line phase or normal navigation
      if (initialStraightPhase) {
        // During straight phase, keep steering centered
        steeringServo.write(STEERING_CENTER + TRIM_ANGLE);
        
        // Exit straight phase after duration elapsed
        if (currentTime - straightPhaseStartTime >= STRAIGHT_PHASE_DURATION) {
          initialStraightPhase = false;
        }
      } else {
        // Normal navigation with steering correction
        int steeringAngle = calculateSteeringAngle(currentLat, currentLon);
        
        // Check if waypoint reached
        if (distance < WAYPOINT_REACHED_RADIUS) {
          handleWaypointReached();
        }
        // steeringAngle = applyObstacleAvoidance(steeringAngle);

        // Apply steering angle
        steeringServo.write(steeringAngle);
      }
      
      // Always update pace and distance in autonomous mode
      updatePaceControl();
      updateDistanceTracking();
      newPVTDataAvailable = false;
    }
  } else {
    // DRIVING STATE - Minimal processing for maximum responsiveness
    // Currently everything is skipped in driving state
  }
  
  delay(1); // Small delay for stability
}
// void loop() {
//   // Process incoming GPS messages
//   myGPS.checkUblox();  // Check for the arrival of new data and process it
//   myGPS.checkCallbacks(); // Check if any callbacks are waiting to be processed
  
//   // Safety checks run regardless of MODE
//   unsigned long currentTime = millis();
  
//   // Safety check: if no clients connected to the AP, stop the vehicle
//   // if (WiFi.softAPgetStationNum() == 0) {
//   //     autonomousMode = false;
//   //     escServo.write(ESC_NEUTRAL);
//   //     steeringServo.write(STEERING_CENTER);
//   // }
//   while (WiFi.status() != WL_CONNECTED) {
//     server.handleClient(); // Process web requests while waiting
//     webSocket.loop();
//     updateWebSocketClients();
//     WiFi.begin(ssid, password); 
//     delay(100);

//     Serial.println("Stop car... Connecting to WiFi...");
//     if (WiFi.status() == WL_CONNECTED) {
//       Serial.printf("Connected to %s\nESP32 IP Address: ", ssid);
//       Serial.println(WiFi.localIP());
//     }
//   }
//   if (!ntripClient.connected()) {
//       connectToNTRIP();
//   }

//   // Try to process RTK connection but don't block if not available
//   bool rtcmStatus = false;
//   if (ntripClient.connected()) {
//     rtcmStatus = processConnection();
//   } else if (WiFi.status() == WL_CONNECTED) {
//     // Try reconnecting to NTRIP occasionally, but don't block
//     static unsigned long lastReconnectAttempt = 0;
//     if (millis() - lastReconnectAttempt > 1000) { // Try every 1 seconds
//       lastReconnectAttempt = millis();
//       connectToNTRIP();
//     }
//   }

//   // Check for command staleness in manual mode
//   if (!autonomousMode && (currentTime - lastCommandTime > COMMAND_TIMEOUT_MS)) {
//     escServo.write(ESC_NEUTRAL);
//     steeringServo.write(STEERING_CENTER);
//   }

//   // handle web requests
//   server.handleClient();
//   webSocket.loop();
//   updateWebSocketClients();

//   // ALWAYS update sonar readings
//    updateSonarReadings();
//   // emergency check for avoidance
//   // int emergencyAovidance = applyObstacleAvoidance(STEERING_CENTER);
  
//   // do autonomous navigation if in auto mode:
//   if (autonomousMode) { //only update status messages in auto mode
//       updateStatusMessages();
//       // only do navigation in auto mode with good GPS fix
//       if (currentFixType > 0 ) {
//           // Calculate distance to target and base steering angle
//           float distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
//           // Check if we're in initial straight-line phase
//           if (initialStraightPhase) {
//               // During straight phase, keep steering centered
//               steeringServo.write(STEERING_CENTER + TRIM_ANGLE);
              
//               // If straight phase duration has elapsed, exit straight phase
//               if ((currentTime >= straightPhaseStartTime) && // BE AWARE: ESP32 HAS DUAL CORE, TIMESTAMPS DISAGREE BY ABOUT 3MS!!
//                   (currentTime - straightPhaseStartTime >= STRAIGHT_PHASE_DURATION) && 
//                   initialStraightPhase) {
//                       initialStraightPhase = false;
//               }
//           } else {
//               // Normal navigation with steering correction
//               int steeringAngle = calculateSteeringAngle(currentLat, currentLon);
              
//               // If the target is reached, handle waypoint or end session
//               if (distance < WAYPOINT_REACHED_RADIUS) {
//                   handleWaypointReached();
//               }
              
//               //steeringAngle = applyObstacleAvoidance(steeringAngle);
              
//               // Constrain and apply the steering angle
//               steeringAngle = constrain(steeringAngle, STEERING_CENTER - STEERING_MAX, STEERING_CENTER + STEERING_MAX);
//               steeringServo.write(steeringAngle);
//           }
          
//           // Update pace control (do this regardless of straight phase)
//           updatePaceControl();
          
//           // Update distance tracking
//           updateDistanceTracking();
          
//           // Reset the new data flag after processing
//           newPVTDataAvailable = false;
//       }
//   }
//   delay(1);
// }
