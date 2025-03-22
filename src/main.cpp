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

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\nRC Car starting up with FreeRTOS...");
    
    // Initialize logging system
    if (initLogging()) {
        LOG_DEBUG("Logging system initialized successfully");
    } else {
        Serial.println("Failed to initialize logging system");
    }

    LOG_ERROR("SYSTEM RESTART DETECTED - this message should appear only once after power-up");
    
    // Connect to WiFi
    LOG_DEBUG("Connecting to WiFi network: %s", ssid);
    WiFi.begin(ssid, password);
    
    // Wait for connection with timeout
    int wifiTimeout = 0;
    while (WiFi.status() != WL_CONNECTED && wifiTimeout < 20) {
        delay(500);
        Serial.print(".");
        wifiTimeout++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        LOG_DEBUG("Connected to WiFi: %s, IP: %s", ssid, WiFi.localIP().toString().c_str());
    } else {
        LOG_ERROR("Failed to connect to WiFi. Will retry in RTOS tasks");
    }
    
    initWebSocket();

    if (!initNavigation()) {
        LOG_ERROR("Failed to initialize navigation system");
    } else {
        LOG_DEBUG("Navigation system initialized successfully");
    }
    
    // Initialize RTOS components
    LOG_DEBUG("Initializing RTOS components");
    initRTOS();

    // Nothing else should run here - all code is now in tasks
    LOG_DEBUG("Setup complete. RTOS scheduler taking over");
}

void loop() {
    // The loop function is not used with FreeRTOS
    // All code should be in task functions
    vTaskDelay(pdMS_TO_TICKS(1000));
}