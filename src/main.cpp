#include <Arduino.h>
#include "config.h"
#include "rtos_tasks.h"
#include "http_server.h"
#include <WiFi.h>
#include "logging.h"
#include "navigation.h"
#include "websocket_handler.h"

// Definition for ntripClient (declared as extern in config.h)
WiFiClient ntripClient;

// Definition for ntripClient mutex (declared as extern in config.h)
SemaphoreHandle_t ntripClientMutex = NULL;

const char* logLevelToString(LogLevel level);

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.printf("\n");
    Serial.printf("\n");
    Serial.printf("\n\nRC Car starting up with FreeRTOS...");
    Serial.printf("\n");
    
    // Initialize logging system
    if (initLogging()) {
        //LOG_DEBUG("logging init");
    } else {
        Serial.printf("logging init fail");
        Serial.printf("\n");
    }

    LOG_ERROR("SYSTEM RESTART DETECTED - this message should appear only once after power-up");
    
    // Connect to WiFi
    Serial.printf("Connecting to WiFi network: %s\n", ssid);
    Serial.printf("\n");
    LOG_DEBUG("Connecting to WiFi network: %s", ssid);
    WiFi.begin(ssid, password);
    
    
    // Wait for connection with timeout
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        char ipStr[16]; // Enough for IPv4: "xxx.xxx.xxx.xxx"
        snprintf(ipStr, sizeof(ipStr), "%u.%u.%u.%u",
            WiFi.localIP()[0], WiFi.localIP()[1],
            WiFi.localIP()[2], WiFi.localIP()[3]);
        Serial.printf("Connected to WiFi: %s, IP: %s\n\n", ssid, ipStr);
        Serial.println(WiFi.localIP());
        LOG_DEBUG("Connected to WiFi: %s, IP: %s", ssid, ipStr);
    } else {
        Serial.printf("Failed to connect to WiFi. Will retry in RTOS tasks");
        Serial.printf("\n");
        LOG_ERROR("Failed to connect to WiFi. Will retry in RTOS tasks");
    }
    
    initWebSocket();

    if (!initNavigation()) {
        LOG_ERROR("Failed to initialize navigation system");
    } else {
        LOG_DEBUG("Navigation system initialized successfully");
    }
    
    // Initialize RTOS components
    LOG_DEBUG("currentLogLevel: %s", logLevelToString(currentLogLevel));
    delay(1000);
    Serial.printf("starting initRTOS");
    Serial.printf("\n");
    initRTOS();

    // Nothing else should run here - all code is now in tasks
    //LOG_DEBUG("Setup complete. RTOS scheduler taking over");
}

void loop() {
}