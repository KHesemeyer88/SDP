#include "rtos_tasks.h"
#include "config.h"
#include <ESP32Servo.h>

// Task handles
TaskHandle_t controlTaskHandle = NULL;
TaskHandle_t websocketTaskHandle = NULL;
TaskHandle_t blinkTaskHandle = NULL;

// Mutex for accessing shared resources
SemaphoreHandle_t servoMutex = NULL;

Servo steeringServo;
Servo escServo;

// Control task - handles servo commands
void ControlTask(void *pvParameters) {
    // Initialize task
    Serial.println("Control Task Started");
    
    // Task loop
    for (;;) {
        // Take the mutex before accessing servos
        if (xSemaphoreTake(servoMutex, portMAX_DELAY) == pdTRUE) {
            // Center servos for now
            steeringServo.write(STEERING_CENTER);
            escServo.write(ESC_NEUTRAL);
            
            // Release the mutex
            xSemaphoreGive(servoMutex);
        }
        
        // Delay for 20ms (50Hz)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// WebSocket task - handles communications
void WebSocketTask(void *pvParameters) {
    // Initialize task
    Serial.println("WebSocket Task Started");
    
    // Task loop
    for (;;) {
        // Simple placeholder - just print a message
        Serial.println("WebSocket task running");
        
        // Delay for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Simple blink task as a sanity check
void BlinkTask(void *pvParameters) {
    // Initialize task
    Serial.println("Blink Task Started");
    
    // Set up the onboard LED
    pinMode(LED_BUILTIN, OUTPUT);
    
    // Task loop
    for (;;) {
        digitalWrite(LED_BUILTIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
        digitalWrite(LED_BUILTIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Initialize all RTOS components
void initRTOS() {
    // Create mutex for servo access
    servoMutex = xSemaphoreCreateMutex();

    // Create tasks
    xTaskCreatePinnedToCore(
        ControlTask,                // Task function
        "ControlTask",              // Task name
        CONTROL_TASK_STACK_SIZE,    // Stack size
        NULL,                       // Parameters
        CONTROL_TASK_PRIORITY,      // Priority
        &controlTaskHandle,         // Task handle
        1                           // Core (1 = Arduino core)
    );
    
    xTaskCreatePinnedToCore(
        WebSocketTask,
        "WebSocketTask",
        WEBSOCKET_TASK_STACK_SIZE,
        NULL,
        WEBSOCKET_TASK_PRIORITY,
        &websocketTaskHandle,
        0
    );
    
    xTaskCreatePinnedToCore(
        BlinkTask,
        "BlinkTask",
        BLINK_TASK_STACK_SIZE,
        NULL,
        BLINK_TASK_PRIORITY,
        &blinkTaskHandle,
        0
    );
}