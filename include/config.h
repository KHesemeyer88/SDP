// config.h
#ifndef CONFIG_H
#define CONFIG_H
// LIDAR pins
#define LIDAR_SDA_PIN 16  // Using previous ECHO_PIN_FRONT
#define LIDAR_SCL_PIN 17  // Using previous TRIGGER_PIN_FRONT
// SD Card pins
#define SD_CS_PIN 12
#define SD_MISO_PIN 14
#define SD_SCK_PIN 27
#define SD_MOSI_PIN 26
#define SD_CD_PIN 25
#define SD_FREQUENCY 20000000  // 20 MHz SPI frequency for SD card

#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "base64.h" //Built-in ESP32 library
#include <WiFiClient.h>

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
const int STEERING_MAX = 65;     // Maximum steering angle deviation
const int TURN_ANGLE = 20;       // Angle to turn 
const int TRIM_ANGLE = 2; //car lists left

#ifndef LED_BUILTIN
#define LED_BUILTIN 2  // GPIO2 is typically the onboard LED on ESP32 DevKitC
#endif

// WiFi settings
// const char* ssid = "RC_Car_Control";
// const char* password = "12345678";
// Hotspot:
//const char ssid[] = "Kians iPhone";
//const char password[] = "Dove'sHamster";

const char ssid[] = "Galaxy XCover FieldPro8858";
const char password[] = "bugo4303";

// MaCORS
const char casterHost[] = "macorsrtk.massdot.state.ma.us"; 
const char casterUser[] = "KHesemeyer88";
const char casterUserPW[] = "kN7?6jtG9YiNMgD@";
const uint16_t casterPort = 32000;
//const char mountPoint[] = "RTCM3MSM_MAGS"; // RTCM 3.2 MSM_MAXX(GNSS) MAGS (Amherst, but maybe change to MABT?)
const char mountPoint[] = "RTCM3MSM_MABN"; // Colrain

// Timing constants
const unsigned long COMMAND_TIMEOUT_MS = 500;
const unsigned long SONAR_UPDATE_INTERVAL = 100; // 100ms between full sonar updates
const unsigned long AVOIDANCE_MESSAGE_TIMEOUT = 1000; // Clear message after 1 second
const unsigned long DESTINATION_MESSAGE_TIMEOUT = 5000;  // 5 seconds

// Navigation constants
const float WAYPOINT_REACHED_RADIUS = 2.0; //meters
const int MAX_WAYPOINTS = 20;
const int NAV_FREQ = 10;

// Sonar filtering
const int FILTER_SAMPLES = 5;  // Number of samples to average

// Waypoint loop settings
//const int DEFAULT_LOOP_COUNT = 1;     // Default number of times to loop through waypoints
const float DEFAULT_TARGET_PACE = 1;  // Default target pace in m/s (0 = no pace control)
const float DEFAULT_TARGET_DISTANCE = 0; // Default target distance in meters (0 = no distance limit)

// Speed control values
const int SPEED_CORRECTION_INTERVAL = 200; // How often to adjust speed for pace (ms)
const float SPEED_CORRECTION_THRESHOLD = 0.05;  // How aggressively to correct speed (0-1)

// RTK Correction status tracking
enum CorrectionStatus {
  CORR_NONE,
  CORR_STALE,
  CORR_FRESH
};

extern WiFiClient ntripClient;

// Mutex for thread-safe access to ntripClient
extern SemaphoreHandle_t ntripClientMutex;

#endif // CONFIG_H