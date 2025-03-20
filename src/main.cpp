#include <Arduino.h>
#include "config.h"
#include "rtos_tasks.h"
#include "http_server.h"
#include <WiFi.h>
#include "logging.h"

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
        LOG_INFO("Logging system initialized successfully");
    } else {
        Serial.println("Failed to initialize logging system");
    }
    
    // Connect to WiFi
    Serial.printf("Connecting to WiFi network: %s\n", ssid);
    LOG_INFO("Connecting to WiFi network: %s", ssid);
    WiFi.begin(ssid, password);
    
    // Wait for connection with timeout
    int wifiTimeout = 0;
    while (WiFi.status() != WL_CONNECTED && wifiTimeout < 20) {
        delay(500);
        Serial.print(".");
        wifiTimeout++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.printf("Connected to %s\n", ssid);
        Serial.print("ESP32 IP Address: ");
        Serial.println(WiFi.localIP());
        LOG_INFO("Connected to WiFi: %s, IP: %s", ssid, WiFi.localIP().toString().c_str());
    } else {
        Serial.println();
        Serial.println("Failed to connect to WiFi. Will retry in RTOS tasks.");
        LOG_ERROR("Failed to connect to WiFi. Will retry in RTOS tasks");
    }
    
    // Initialize RTOS components
    LOG_INFO("Initializing RTOS components");
    initRTOS();
    
    // Nothing else should run here - all code is now in tasks
    Serial.println("Setup complete. RTOS scheduler taking over.");
    LOG_INFO("Setup complete. RTOS scheduler taking over");
}

void loop() {
    // The loop function is not used with FreeRTOS
    // All code should be in task functions
    vTaskDelay(pdMS_TO_TICKS(1000));
}