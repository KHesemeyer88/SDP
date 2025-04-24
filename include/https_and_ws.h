#ifndef HTTPS_AND_WS_H
#define HTTPS_AND_WS_H

#include <HTTPSServer.hpp>
#include <SSLCert.hpp>
#include <HTTPRequest.hpp>
#include <HTTPResponse.hpp>
#include <ResourceNode.hpp>
#include <WebsocketHandler.hpp>
#include <sstream>
#include <vector>

#include "config.h"
#include "rtos_tasks.h"  


using namespace httpsserver;

// SSLCert* cert = new SSLCert(
//     (unsigned char*)server_cert, strlen(server_cert),
//     (unsigned char*)server_key, strlen(server_key)
// );
// HTTPSServer secureServer(cert);
extern SSLCert* cert;
extern HTTPSServer secureServer;
const int MAX_CLIENTS = 4;
class GNSSWebSocketHandler;
extern std::vector<GNSSWebSocketHandler*> activeClients;
// std::vector<GNSSWebSocketHandler*> activeClients;


class GNSSWebSocketHandler : public httpsserver::WebsocketHandler {
public:
    static httpsserver::WebsocketHandler* create();

    void onMessage(httpsserver::WebsocketInputStreambuf* input) override;
    void onClose() override;

    void sendGNSSBinary();
    void sendRTKBinary();
    void sendNAVBinary();
    void sendSENSORBinary();
    void sendWAYPOINTBinary();
};



// ----- esp32 to webpage -----
struct __attribute__((packed)) GNSS_data {
    uint8_t struct_id = 1;
    float lat = 0;
    float lon = 0;
    uint8_t gnss_fix_type;
};
struct __attribute__((packed)) RTK_status {
    uint8_t struct_id = 2;
    uint8_t RTK_correction_status;
    unsigned long correction_age;
    uint8_t connection_status;
    uint8_t RTK_carrier_solution;
    double h_accuracy;
    uint8_t gnss_fix_type;
};
struct __attribute__((packed)) Nav_stats {
    uint8_t struct_id = 3;
    float total_distance;
    float current_pace;
    float average_pace;
    unsigned long total_time;
};
struct __attribute__((packed)) Sensor_data {
    uint8_t struct_id = 4;
    float lidar_left;
    float lidar_front;
    float lidar_right;
    uint8_t object_detected = false;
};
struct __attribute__((packed)) Waypoint_data {
    uint8_t struct_id = 5;
    float lat;
    float lon;
    uint8_t count;
};

// ----- webpage to esp32 messages -----
const uint8_t MESSAGE_STOP = 0;
const uint8_t MESSAGE_RECORD = 1;
const uint8_t MESSAGE_CLEAR = 2;
//const uint8_t MESSAGE_START = 3;
const uint8_t MESSAGE_PAUSE = 4;
const uint8_t MESSAGE_RESUME = 5;
const uint8_t MESSAGE_RESET = 6;
// ----- webpage to esp32 data -----
const uint8_t COMMAND_CONTROL_ID = 248;
struct __attribute__((packed)) command_control {
    uint8_t id;
    float joystick_y;
    float joystick_x;
};
const uint8_t COMMAND_START_ID = 249;
struct __attribute__((packed)) command_start {
    uint8_t id;
    float target_pace;
    float target_distance;
    float lat = 0;
    float lon;
};
const uint8_t PHONE_POSITION_ID = 250;
struct __attribute__((packed)) phone_position {
    uint8_t id;
    float phone_lat;
    float phone_lon;
    float phone_speed;
};
   

extern GNSS_data my_gnss_data;
extern RTK_status my_rtk_status;
extern Nav_stats my_nav_stats;
extern Sensor_data my_sensor_data;
extern Waypoint_data my_waypoint_data;
extern phone_position my_phone_position;

// Timing variables for updates
extern unsigned long lastWSSensorUpdate;
extern unsigned long lastWSGPSUpdate;
extern unsigned long lastWSRTKUpdate;
extern unsigned long lastWSStatsUpdate;

// ----- Functions defined in https_and_ws.cpp -----
void initWebSocket();
void handleRoot(HTTPRequest* req, HTTPResponse* res);
void cleanupWebSockets();
void WebSocketTask(void *pvParameters);
void sendSensorData();
void sendGPSData();
void sendRTKStatus();
void sendNavigationStats();
void sendStatusMessage(const String& message);
void sendErrorMessage(const String& message);

#endif // HTTPS_AND_WS_H