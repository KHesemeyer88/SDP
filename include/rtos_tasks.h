#ifndef RTOS_TASKS_H
#define RTOS_TASKS_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"
#include "http_server.h"
#include <ESP32Servo.h>

// Task priorities (1-24, higher number = higher priority)
#define CONTROL_TASK_PRIORITY       10
#define WEBSOCKET_TASK_PRIORITY     7
#define BLINK_TASK_PRIORITY         1

// Task stack sizes
#define CONTROL_TASK_STACK_SIZE     4096
#define WEBSOCKET_TASK_STACK_SIZE   16384
#define BLINK_TASK_STACK_SIZE       2048
// HTTP server task priority and stack size
#define HTTP_SERVER_TASK_PRIORITY     8
#define HTTP_SERVER_TASK_STACK_SIZE   8192

// HTTP server task handle
extern TaskHandle_t httpServerTaskHandle;

// Forward declaration of HTTP server task
void HttpServerTask(void *pvParameters);

// Command types for the control queue
enum CommandType {
    CMD_MANUAL_CONTROL,
    CMD_AUTO_CONTROL,
    CMD_STOP
};

// Command structure for manual control
struct ManualControlCommand {
    float throttle;    // -1 to 1, negative is reverse
    float steering;    // -1 to 1, negative is left
    bool isDriving;    // Driving state flag
};

// Command structure for autonomous control
struct AutoControlCommand {
    float targetLat;
    float targetLon;
    bool followWaypoints;
};

// Union of all command types
struct ControlCommand {
    CommandType type;
    union {
        ManualControlCommand manual;
        AutoControlCommand autonomous;
    };
};

// Task handles
extern TaskHandle_t controlTaskHandle;
extern TaskHandle_t websocketTaskHandle;
extern TaskHandle_t blinkTaskHandle;

// Mutex for accessing shared resources
extern SemaphoreHandle_t servoMutex;

// Queue for passing commands from WebSocket to control task
extern QueueHandle_t commandQueue;

// Servo objects
extern Servo steeringServo;
extern Servo escServo;

// Task functions
void ControlTask(void *pvParameters);
// WebSocket task now only handles periodic updates, not the WebSocket loop()
void WebSocketTask(void *pvParameters);
void BlinkTask(void *pvParameters);

// Initialize all RTOS components
void initRTOS();

#endif // RTOS_TASKS_H