#include "rtos_tasks.h"
#include "config.h"
#include <ESP32Servo.h>

// Task handles
TaskHandle_t controlTaskHandle = NULL;
TaskHandle_t websocketTaskHandle = NULL;
TaskHandle_t blinkTaskHandle = NULL;

// Mutex for accessing shared resources
SemaphoreHandle_t servoMutex = NULL;

// Queue for passing commands from WebSocket to control task
QueueHandle_t commandQueue = NULL;

// Servo objects - these need to be defined here
Servo steeringServo;
Servo escServo;

// Forward declarations of functions implemented in other files
void WebSocketTask(void *pvParameters);
void ControlTask(void *pvParameters);

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
    Serial.println("Initializing RTOS components...");
    
    // Create mutex for servo access
    servoMutex = xSemaphoreCreateMutex();
    if (servoMutex == NULL) {
        Serial.println("Failed to create servo mutex!");
    }
    
    // Create command queue
    commandQueue = xQueueCreate(10, sizeof(ControlCommand));
    if (commandQueue == NULL) {
        Serial.println("Failed to create command queue!");
    }
    
    // Initialize servos
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    
    steeringServo.setPeriodHertz(50);
    escServo.setPeriodHertz(50);
    
    steeringServo.attach(STEERING_PIN, 1000, 2000);
    escServo.attach(ESC_PIN, 1000, 2000);
    
    // Initialize ESC to neutral
    escServo.write(ESC_NEUTRAL);
    delay(100);
    steeringServo.write(STEERING_CENTER);
    delay(100);
    
    Serial.println("Servos initialized");
    
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
    
    Serial.println("RTOS tasks created");
}