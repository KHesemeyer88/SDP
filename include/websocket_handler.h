#ifndef WEBSOCKET_HANDLER_H
#define WEBSOCKET_HANDLER_H

#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h> 
#include <ArduinoJson.h>
#include "config.h"
#include "rtos_tasks.h"  

// WebSocket configuration
const size_t JSON_CAPACITY = 512;  // Adjust based on your data needs

// WebSocket update intervals (keep the same as before)
const unsigned long WS_SENSOR_UPDATE_INTERVAL = 500;   // 100ms for sensor data
const unsigned long WS_GPS_UPDATE_INTERVAL = 500;      // 200ms for GPS data
const unsigned long WS_RTK_UPDATE_INTERVAL = 2000;     // 1000ms for RTK status
const unsigned long WS_STATS_UPDATE_INTERVAL = 500;    // 500ms for navigation stats

// Timing variables for updates
extern unsigned long lastWSSensorUpdate;
extern unsigned long lastWSGPSUpdate;
extern unsigned long lastWSRTKUpdate;
extern unsigned long lastWSStatsUpdate;

// Forward declare AsyncWebSocket
class AsyncWebSocket;
extern AsyncWebSocket ws;

// Function to handle incoming WebSocket JSON messages
void webSocketEventRTOS(AsyncWebSocket *server, AsyncWebSocketClient *client, 
    AwsEventType type, void *arg, uint8_t *data, size_t len);

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