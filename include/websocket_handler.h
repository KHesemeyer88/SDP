#ifndef WEBSOCKET_HANDLER_H
#define WEBSOCKET_HANDLER_H

#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "config.h"
#include "rtos_tasks.h"  

// WebSocket configuration
const size_t JSON_CAPACITY = 512;  // Adjust based on your data needs

// WebSocket update intervals
const unsigned long WS_SENSOR_UPDATE_INTERVAL = 100;   // 100ms for sensor data
const unsigned long WS_GPS_UPDATE_INTERVAL = 200;      // 200ms for GPS data
const unsigned long WS_RTK_UPDATE_INTERVAL = 1000;     // 1000ms for RTK status
const unsigned long WS_STATS_UPDATE_INTERVAL = 500;    // 500ms for navigation stats

// Timing variables for updates
extern unsigned long lastWSSensorUpdate;
extern unsigned long lastWSGPSUpdate;
extern unsigned long lastWSRTKUpdate;
extern unsigned long lastWSStatsUpdate;

// Forward declare the WebSocket server
class WebSocketsServer;
extern WebSocketsServer webSocket;

// WebSocket event handler for RTOS environment
void webSocketEventRTOS(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

// Function to send sensor data via WebSocket
void sendSensorData(uint8_t clientNum = 255);

// Function to send GPS data via WebSocket
void sendGPSData(uint8_t clientNum = 255);

// Function to send RTK status via WebSocket  
void sendRTKStatus(uint8_t clientNum = 255);

// Function to send navigation stats via WebSocket
void sendNavigationStats(uint8_t clientNum = 255);

// Function to send a status message
void sendStatusMessage(const String& message);

// Function to send an error message
void sendErrorMessage(const String& message);

// WebSocket task function (declared in rtos_tasks.h, implemented in websocket_handler.cpp)
void WebSocketTask(void *pvParameters);

#endif // WEBSOCKET_HANDLER_H