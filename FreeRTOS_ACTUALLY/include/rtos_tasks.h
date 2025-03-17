#ifndef RTOS_TASKS_H
#define RTOS_TASKS_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"

// Task priorities (1-24, higher number = higher priority)
#define CONTROL_TASK_PRIORITY       10
#define WEBSOCKET_TASK_PRIORITY     8
#define BLINK_TASK_PRIORITY         5

// Task stack sizes
#define CONTROL_TASK_STACK_SIZE     4096
#define WEBSOCKET_TASK_STACK_SIZE   8192
#define BLINK_TASK_STACK_SIZE       2048

// Task handles
extern TaskHandle_t controlTaskHandle;
extern TaskHandle_t websocketTaskHandle;
extern TaskHandle_t blinkTaskHandle;

// Task functions
void ControlTask(void *pvParameters);
void WebSocketTask(void *pvParameters);
void BlinkTask(void *pvParameters);

// Mutex for accessing shared resources
extern SemaphoreHandle_t servoMutex;

// Initialize all RTOS components
void initRTOS();

#endif // RTOS_TASKS_H