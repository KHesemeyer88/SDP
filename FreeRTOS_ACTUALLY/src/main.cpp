#include <Arduino.h>
#include "config.h"
#include "rtos_tasks.h"
WiFiClient ntripClient;
// Retain your global variables and initialization functions

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\nRC Car starting up with FreeRTOS...");
    
    // Initialize hardware
    // ... your existing initialization code ...
    
    // Initialize RTOS components
    initRTOS();
    
    // Nothing else should run here - all code is now in tasks
    Serial.println("Setup complete. RTOS scheduler taking over.");
}

void loop() {
    // The loop function is not used with FreeRTOS
    // All code should be in task functions
    vTaskDelay(pdMS_TO_TICKS(1000));
}