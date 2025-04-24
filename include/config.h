#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>

#include <HTTPSServer.hpp>
#include <SSLCert.hpp>
#include <HTTPRequest.hpp>
#include <HTTPResponse.hpp>
#include <ResourceNode.hpp>
#include <WebsocketHandler.hpp>
#include <sstream>
#include <vector>

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
const char ssid[] = "Kians iPhone";
const char password[] = "Dove'sHamster";

// const char ssid[] = "Galaxy XCover FieldPro8858";
// const char password[] = "bugo4303";

// const char ssid[] = "chewchewchew";
// const char password[] = "03092738Ss";

// const char ssid[] = "Kians iPhone";
// const char password[] = "Dove'sHamster";
// const char ssid2[] = "Galaxy XCover FieldPro8858";
// const char password2[] = "bugo4303";

// MaCORS
const char casterHost[] = "macorsrtk.massdot.state.ma.us"; 
const char casterUser[] = "KHesemeyer88";
const char casterUserPW[] = "kN7?6jtG9YiNMgD@";
const uint16_t casterPort = 32000;
//const char mountPoint[] = "RTCM3MSM_MAGS"; // RTCM 3.2 MSM_MAXX(GNSS) MAGS (Amherst, but maybe change to MABT?)
//const char mountPoint[] = "RTCM3MSM_MABN"; // Colrain
const char mountPoint[] = "RTCM3MSM_MABT"; // Amherst


// Certs
const char server_cert[] PROGMEM = R"rawliteral(
-----BEGIN CERTIFICATE-----
MIIDlzCCAn+gAwIBAgIUMeHTVt74QOrix3eoUydY0ig/+VkwDQYJKoZIhvcNAQEL
BQAwWzELMAkGA1UEBhMCQVUxEzARBgNVBAgMClNvbWUtU3RhdGUxITAfBgNVBAoM
GEludGVybmV0IFdpZGdpdHMgUHR5IEx0ZDEUMBIGA1UEAwwLMTcyLjIwLjEwLjIw
HhcNMjUwNDIyMjAxNjQyWhcNMjYwNDIyMjAxNjQyWjBbMQswCQYDVQQGEwJBVTET
MBEGA1UECAwKU29tZS1TdGF0ZTEhMB8GA1UECgwYSW50ZXJuZXQgV2lkZ2l0cyBQ
dHkgTHRkMRQwEgYDVQQDDAsxNzIuMjAuMTAuMjCCASIwDQYJKoZIhvcNAQEBBQAD
ggEPADCCAQoCggEBANJcGIQUpIVv3/Yr+ktUkujJNNkhxJC8Ghuy5bsYUTtj5Rmz
PFn/adAADx8o1Y+zh64SWZeak3y+f9/xtoJSAjG7/HSvlFXCYayMzV/FMejPrvd7
K+XC07hdREI1e3Tufoz362uI/bJ7csL8K+nbq0ZUZheCw/sIjSsillhWVQw3weh7
iDI3Zx2ZQADKvqIMZ+Ww1HGap/2N2ZczzREKJBFrm90V0X/Dpx4JmW4MSlDYnLak
UbmqIZMMBdzkHPS0LBBdiSHN+B5ox0bIXCtyOyY+qPX+2dEkGtwvyHUFktTw4fXF
I0rREepKPYVIVuBgN4Oh+gfLoob4wFzUVJzwrM0CAwEAAaNTMFEwHQYDVR0OBBYE
FNyC81nZmNrPc7q48X9bCc+mTHvDMB8GA1UdIwQYMBaAFNyC81nZmNrPc7q48X9b
Cc+mTHvDMA8GA1UdEwEB/wQFMAMBAf8wDQYJKoZIhvcNAQELBQADggEBAMKd+Wnk
mE2HF/jlR++Jixv7kLFmEOPPzItgCx+A0cf+WP5BeCDwIGalYeoJzuU4T3395IzH
NfkaeKFIbqptgzKMGPCNXmS23LgIqwMWxI5FTzEoa2AqfHzVDqInmqo7hZmwKmHE
hhfodjrWBMxnpCHNLw+tgPTMV/Xesfk6IU/kYpQFt3aV2rXon6qQsopk34UgOLQr
CBGIJSvrP0cSGuU66nmaC7ay8pkBtOoiTFhKgjS4jDLgJ6/Em2texfVhEnEa7MAM
M5ZkScOyjypPNyMdk2/MyZ57BNiIIdWUMaVFXemHghQTv3rLRe2E5SsAbH5/uDga
wKe85aR10oS6WS4=
-----END CERTIFICATE-----
)rawliteral";
    
const char server_key[] PROGMEM = R"rawliteral(
-----BEGIN PRIVATE KEY-----
MIIEvQIBADANBgkqhkiG9w0BAQEFAASCBKcwggSjAgEAAoIBAQDSXBiEFKSFb9/2
K/pLVJLoyTTZIcSQvBobsuW7GFE7Y+UZszxZ/2nQAA8fKNWPs4euElmXmpN8vn/f
8baCUgIxu/x0r5RVwmGsjM1fxTHoz673eyvlwtO4XURCNXt07n6M9+triP2ye3LC
/Cvp26tGVGYXgsP7CI0rIpZYVlUMN8Hoe4gyN2cdmUAAyr6iDGflsNRxmqf9jdmX
M80RCiQRa5vdFdF/w6ceCZluDEpQ2Jy2pFG5qiGTDAXc5Bz0tCwQXYkhzfgeaMdG
yFwrcjsmPqj1/tnRJBrcL8h1BZLU8OH1xSNK0RHqSj2FSFbgYDeDofoHy6KG+MBc
1FSc8KzNAgMBAAECggEAAPctGmVFzSnBocYdG3nZgRFo4sqp0Czm9/PIzWwGoxJP
cNivKKhMJzofMf/nSnNBG06B0GJtu6sJaSHdI4Rf4A6I77ZZmX4rqYL1K9WkPnWn
z5kijxub+34jKFkraC0Bpsh9xCZt845JPEiOHXTeDyprNjCPtt6gT5rFdpDWDdVH
fhdl/MX3adp4zYE7XWjc992zYMyBnea4/mZGUC9rKyLKTrpXxY4l4OcTdiFE+E/7
jbEuqe/iZHPys9bKOg66LxWd54diD5D1aBbDm+apQ3ODil+7Xr+nF7pm6scNLd6t
D8U7qmYpy9nY5m9GmA7AwzpN8UBq4zwBfm43qnB1sQKBgQDyrq3IkKPAt4SfjElA
6kQ9hWizSz9hM1P/YBhnDEGfEnSneganqYZlTG1NoiD6B16YJvgYSIoKVrpDzS32
snFZ9+sKTnzbuKtFp5XslBX8Q9Lh0tYfcI5+LnVxw7s49DGsf3E8cFtsGCHH+tAu
DI89FDs8m23NDaByHDyXu6yC3QKBgQDd51VPP38KgvaYSjmJ/NKE/e48u9/ymm3a
6Yv0gG/pImSZXEK21UTVyz3L1xyTiXuInINVnMM/Vdx1L56QkXjE0HdcX0F9m/PS
Gs2E/VFRRhtjArAZLAnThZiMh9dvYjDEq4ebUubrgaNCE2/jNYQDGYCAc4G/MVKU
ZFtVC4LasQKBgAFqHz0yCqJO74j2il7Efs1U/707zQzF/dFZQAspuSAyPVfUkEFd
7Zidj22KamLKtDRy1bNeiN9yjfdjNMdhVuPNXCNdPcESHH11cpxEaLRluM70KadZ
QptdrfWRzH+SfM+ilohhp0bEBT5jKd9610Ll4UqDJWqyj6Lao0fHIkRJAoGARB/5
ONMtlXSPcEGZWJudpeavdUXYgwqWH1cD+JRVxyUf7VU9xhPDhj9XQXVQ+JqEqnQi
fe4aox8hB3kPHSMMCKBXhKxZ1s5CFIbWAbYjeOglEEiK969ldOLW6o3pvfBPOJHQ
mAbjjfnGvUpqVz7ewHTb1pOfiasCxzvoLjvSeIECgYEAzwaYAxlosmO4Laon1qyd
mTnA1Nz5GK1OLGoTRu5D+Y0gIWVZl1Fi1L/WmykC5UBWZmpYqLcvo8JKbXvTYoOk
nrjO5z6x4MIFfvDP/HsiqmQ25F9YGyOJzOhs9KuZpIbdJ6+yaRTaEwKr5VhZwro1
5WjlZ91HUD0PA9YIaigogDk=
-----END PRIVATE KEY-----
)rawliteral";

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