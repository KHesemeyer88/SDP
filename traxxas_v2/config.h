// config.h
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SparkFun_u-blox_GNSS_v3.h>

// Pin Definitions
const int STEERING_PIN = 5;   // GPIO5 for steering servo
const int ESC_PIN = 23;       // GPIO23 for ESC control
// Sonar pins
const int TRIGGER_PIN_FRONT = 17;
const int ECHO_PIN_FRONT = 16;
const int TRIGGER_PIN_LEFT = 22;
const int ECHO_PIN_LEFT = 18;
const int TRIGGER_PIN_RIGHT = 19;
const int ECHO_PIN_RIGHT = 21;

// Sonar thresholds (cm)
const int FRONT_STOP_THRESHOLD = 150; // zero to suppress obst. avoidance
const int SIDE_AVOID_THRESHOLD = 50;

// Traxxas XL-2.5 ESC & servo values. Note ESC takes angles like a servo, but converts them to speed
const int ESC_NEUTRAL = 90;     // Neutral position (1.5ms pulse)
const int ESC_MAX_FWD = 150;    // Max forward allowed (180 removes speed governor)
const int ESC_MAX_REV = 50;     // Max reverse allowed (~1.1ms pulse)
const int ESC_MIN_FWD = 95;     // Minimum forward throttle
const int ESC_MIN_REV = 85;     // Minimum reverse throttle
const int STEERING_CENTER = 90;  // Center steering
const int STEERING_MAX = 55;     // Maximum steering angle deviation
const int TURN_ANGLE = 20;       // Angle to turn 

// WiFi settings
const char* ssid = "RC_Car_Control";
const char* password = "12345678";

// Timing constants
const unsigned long COMMAND_TIMEOUT_MS = 500;
const unsigned long SONAR_UPDATE_INTERVAL = 100; // 100ms between full sonar updates
const unsigned long AVOIDANCE_MESSAGE_TIMEOUT = 1000; // Clear message after 1 second
const unsigned long DESTINATION_MESSAGE_TIMEOUT = 5000;  // 5 seconds

// Navigation constants
const float WAYPOINT_REACHED_RADIUS = 2.0;  // 2 meters radius
const int MAX_WAYPOINTS = 20;
const int NAV_FREQ = 30;

// Sonar filtering
const int FILTER_SAMPLES = 5;  // Number of samples to average

// Global variables
extern WebServer server;
extern SFE_UBLOX_GNSS myGPS;
extern Servo steeringServo, escServo;

// GPS and waypoints
extern float targetLat, targetLon;
extern float waypointLats[], waypointLons[];
extern int waypointCount, currentWaypointIndex;
extern bool followingWaypoints, autonomousMode, destinationReached;
extern volatile float currentLat, currentLon;
extern volatile float currentSpeed;
extern volatile uint8_t currentFixType;
extern volatile bool newPVTDataAvailable;

// Sensor readings
extern float lastFrontDist, lastLeftDist, lastRightDist;
extern String lastAvoidanceMessage;

// Timing variables
extern unsigned long lastSonarUpdate;
extern unsigned long lastAvoidanceTime, destinationReachedTime;

// HTML content for webpage
extern const char webPage[] PROGMEM;

// Waypoint loop settings
//const int DEFAULT_LOOP_COUNT = 1;     // Default number of times to loop through waypoints
const float DEFAULT_TARGET_PACE = 1;  // Default target pace in m/s (0 = no pace control)
const float DEFAULT_TARGET_DISTANCE = 0; // Default target distance in meters (0 = no distance limit)

// Speed control values
const int SPEED_CORRECTION_INTERVAL = 50; // How often to adjust speed for pace (ms)
const float SPEED_CORRECTION_THRESHOLD = 0.05;  // How aggressively to correct speed (0-1)

// Extended extern declarations for new tracking variables
//extern int waypointLoopCount;         // Current count of completed loops
//extern int targetLoopCount;           // Target number of loops to complete
extern float targetPace;              // Target pace in m/s
extern float targetDistance;          // Target total distance in meters
extern float totalDistance;           // Total distance traveled so far
extern unsigned long totalTimeMs;     // Total time elapsed in ms
extern float currentPace;             // Current pace in m/s
extern unsigned long lastPaceUpdate;  // Last time pace was calculated
extern float lastSegmentDistance;     // Distance of last segment for pace calculation
extern float lastTrackedLat;
extern float lastTrackedLon; 
extern unsigned long lastDistanceUpdate;

// Straight phase variables
extern bool initialStraightPhase;
extern unsigned long straightPhaseStartTime;
extern const unsigned long STRAIGHT_PHASE_DURATION;

#endif // CONFIG_H