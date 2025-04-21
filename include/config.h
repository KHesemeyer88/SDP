#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "base64.h" //Built-in ESP32 library
#include <WiFiClient.h>


// config.h
#ifndef CONFIG_H
#define CONFIG_H
// LIDAR pins
#define LIDAR_SDA_PIN 16
#define LIDAR_SCL_PIN 17
// SD Card SPI pins
#define SD_CS_PIN 18
#define SD_MISO_PIN 19
#define SD_SCK_PIN 21
#define SD_MOSI_PIN 22
#define SD_CD_PIN 25
#define SD_FREQUENCY 20000000  // 20 MHz SPI frequency for SD card
// Navigation board SPI pins
#define NAV_CS_PIN    15     // Chip Select (active low)
#define NAV_MOSI_PIN  13   // COPI (Controller Out, Peripheral In)
#define NAV_MISO_PIN  12   // CIPO (Controller In, Peripheral Out)
#define NAV_SCK_PIN   14    // SCLK (Serial Clock)
#define NAV_INT_PIN   26    // Interrupt pin
#define NAV_RST_PIN   27    // Reset pin
#define NAV_SPI_FREQUENCY 4000000  // 20 MHz SPI frequency - drop down if unstable

// Timing constants
#define COMMAND_TIMEOUT_MS          500
#define SONAR_UPDATE_INTERVAL       50
#define AVOIDANCE_MESSAGE_TIMEOUT   1000
#define DESTINATION_MESSAGE_TIMEOUT 5000
#define CONTROL_UPDATE_FREQUENCY    NAV_FREQ
#define NAV_UPDATE_FREQUENCY        NAV_FREQ


// WebSocket update intervals
const unsigned long WS_SENSOR_UPDATE_INTERVAL = 500;
const unsigned long WS_GPS_UPDATE_INTERVAL = 500;
const unsigned long WS_RTK_UPDATE_INTERVAL = 2000;
const unsigned long WS_STATS_UPDATE_INTERVAL = 500;

// Pin Definitions
const int STEERING_PIN = 5;   // GPIO5 for steering servo
const int ESC_PIN = 23;       // GPIO23 for ESC control

// Sonar thresholds (cm)
const int FRONT_STOP_THRESHOLD = 300; // in cm
//const int SIDE_AVOID_THRESHOLD = 50;

// Traxxas XL-2.5 ESC & servo values. Note ESC takes angles like a servo, but converts them to speed
const int ESC_NEUTRAL = 90;     // Neutral position (1.5ms pulse)
const int ESC_MAX_FWD = 150;    // Max forward allowed (180 removes speed governor)
const int ESC_MAX_REV = 50;     // Max reverse allowed (~1.1ms pulse)
const int ESC_MIN_FWD = 95;     // Minimum forward throttle
const int ESC_MIN_REV = 85;     // Minimum reverse throttle
const int STEERING_CENTER = 90;  // Center steering
const int STEERING_MAX = 65;     // Maximum steering angle deviation
const int TURN_ANGLE = 40;       // Angle to turn 
const int TRIM_ANGLE = 2;       //car lists left

// WiFi settings
// const char* ssid = "RC_Car_Control";
// const char* password = "12345678";
// Hotspot:
// const char ssid[] = "Kians iPhone";
// const char password[] = "Dove'sHamster";

const char ssid[] = "Galaxy XCover FieldPro8858";
const char password[] = "bugo4303";

// const char ssid[] = "chewchewchew";
// const char password[] = "03092738Ss";

// MaCORS
const char casterHost[] = "macorsrtk.massdot.state.ma.us"; 
const char casterUser[] = "KHesemeyer88";
const char casterUserPW[] = "kN7?6jtG9YiNMgD@";
const uint16_t casterPort = 32000;
//const char mountPoint[] = "RTCM3MSM_MAGS"; // RTCM 3.2 MSM_MAXX(GNSS) MAGS (Amherst, but maybe change to MABT?)
//const char mountPoint[] = "RTCM3MSM_MABN"; // Colrain
const char mountPoint[] = "RTCM3MSM_MABT"; // Amherst

// Navigation constants
const float WAYPOINT_REACHED_RADIUS = 2.0; //meters
const int MAX_WAYPOINTS = 20;
const int NAV_FREQ = 20;

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

// Function to handle system errors
void handleSystemError(const char* errorMsg, bool restartSystem);
void cleanupResources(bool cleanupMutexes, bool cleanupQueues, bool cleanupTasks);

extern WiFiClient ntripClient;

// Mutex for thread-safe access to ntripClient
extern SemaphoreHandle_t ntripClientMutex;

#endif // CONFIG_H