#ifndef WEBSOCKET_HANDLER_H
#define WEBSOCKET_HANDLER_H

#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h> 
//#include <ArduinoJson.h>
#include "config.h"
#include "rtos_tasks.h"  

// WebSocket configuration

// case 1: fixDesc = "1 (Dead Reckoning)"; break;
// case 2: fixDesc = "2 (2D)"; break;
// case 3: fixDesc = "3 (3D)"; break;
// case 4: fixDesc = "4 (GNSS+DR)"; break;
// case 5: fixDesc = "5 (Time Only)"; break;
// default: fixDesc = "Unknown"; break;

// struct MessageHeader {
//     uint8_t type;    // e.g. 1 = GNSSData, 2 = CommandData, 3 = ConfigData
//     uint8_t length;  // optional, for variable-length payloads
// };




// ------ from esp32
const uint8_t GNSS_ID = 1;
//etc... 
struct GNSS_data {
    uint8_t struct_id = GNSS_ID;
    float lat = 0;
    float lon = 0;
    uint8_t gnss_fix_type;
};
struct RTK_status {
    uint8_t struct_id = 2;
    uint8_t RTK_correction_status;
    unsigned long correction_age;
    uint8_t connection_status;
    uint8_t RTK_carrier_solution;
    double h_accuracy;
    uint8_t gnss_fix_type;
};
struct Nav_stats {
    uint8_t struct_id = 3;
    float total_distance;
    float current_pace;
    float average_pace;
    unsigned long total_time;
};
struct Sensor_data {
    uint8_t struct_id = 4;
    float lidar_left;
    float lidar_front;
    float lidar_right;
    uint8_t object_detected = false;
};
struct Waypoint_data {
    uint8_t struct_id = 5;
    float lat;
    float lon;
    uint8_t count;
};
// struct Auto_mode {
//     uint8_t struct_id = 6;
//     uint8_t auto_mode;
// };




// ----- from webpage

struct Recieved_Waypoint {
    float lon;
    float lat;
};

struct command_start {
    uint8_t id;
    float target_pace;
    float target_distance;
    float lat = 0;
    float lon;
};
struct command_control {
    uint8_t id;
    float joystick_y;
    float joystick_x;
};


const uint8_t MESSAGE_STOP = 0;
const uint8_t MESSAGE_RECORD = 1;
const uint8_t MESSAGE_CLEAR = 2;
//const uint8_t MESSAGE_START = 3;
const uint8_t MESSAGE_PAUSE = 4;
const uint8_t MESSAGE_RESUME = 5;
const uint8_t MESSAGE_RESET = 6;

const uint8_t COMMAND_START = 2;
const uint8_t COMMAND_CONTROL = 1;


//          ws.sendMessage(1);




// struct websocket_message_from_webpage {
//     uint8_t manual_mode = true;

// };

// // extern volatile websocket_message_from_ESP32 my_esp32_data;
// extern websocket_message_from_webpage my_webpage_data;




// Timing variables for updates
extern unsigned long lastWSSensorUpdate;
extern unsigned long lastWSGPSUpdate;
extern unsigned long lastWSRTKUpdate;
extern unsigned long lastWSStatsUpdate;

// Forward declare AsyncWebSocket
class AsyncWebSocket;
extern AsyncWebSocket ws;

// Function to handle incoming WebSocket JSON messages
void webSocketEventHandler(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

// Function to send sensor data via WebSocket
void sendSensorData(AsyncWebSocketClient *client = nullptr);

// Function to send GPS data via WebSocket
void sendGPSData(AsyncWebSocketClient *client = nullptr);

// Function to send RTK status via WebSocket  
void sendRTKStatus(AsyncWebSocketClient *client = nullptr);

// Function to send navigation stats via WebSocket
void sendNavigationStats(AsyncWebSocketClient *client = nullptr);

// Function to send a status message
void sendStatusMessage(const String& message);

// Function to send an error message
void sendErrorMessage(const String& message);

// WebSocket task function - manages periodic updates
void WebSocketTask(void *pvParameters);

// Clean disconnected clients (called periodically)
void cleanupWebSockets();

// Initialize the WebSocket server (called from HttpServer initialization)
void initWebSocket();

#endif // WEBSOCKET_HANDLER_H