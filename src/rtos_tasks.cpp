#include "rtos_tasks.h"
#include "http_server.h"
#include "webpage.h"
#include "config.h"
#include <ESP32Servo.h>
#include "gnss.h"
#include "logging.h"
#include "navigation.h"

// Task handles
TaskHandle_t controlTaskHandle = NULL;
TaskHandle_t websocketTaskHandle = NULL;
TaskHandle_t httpServerTaskHandle = NULL;

// Mutex for accessing shared resources
SemaphoreHandle_t servoMutex = NULL;

// Queue for passing commands from WebSocket to control task
QueueHandle_t commandQueue = NULL;

// Servo objects
Servo steeringServo;
Servo escServo;

// Forward declarations of functions implemented in other files
void WebSocketTask(void *pvParameters);
void ControlTask(void *pvParameters);

// Initialize all RTOS components
void initRTOS() {
    LOG_DEBUG("Initializing RTOS components...");
    
    // Create mutex for servo access
    servoMutex = xSemaphoreCreateMutex();
    if (servoMutex == NULL) {
        handleSystemError("Failed to create servo mutex", true);
        return;
    }

    // Create mutex for GNSS data access
    gnssMutex = xSemaphoreCreateMutex();
    if (gnssMutex == NULL) {
        handleSystemError("Failed to create GNSS mutex", true);
        return;
    }

    // Create mutex for ntripClient access
    ntripClientMutex = xSemaphoreCreateMutex();
    if (ntripClientMutex == NULL) {
        handleSystemError("Failed to create ntripClient mutex", true);
        return;
    }
    
    // Create command queue
    commandQueue = xQueueCreate(10, sizeof(ControlCommand));
    if (commandQueue == NULL) {
        handleSystemError("Failed to create command queue", true);
        return; 
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
    
    LOG_DEBUG("Servos initialized");
    
    // Create tasks
    BaseType_t xReturned = xTaskCreatePinnedToCore(
        ControlTask,                // Task function
        "ControlTask",              // Task name
        CONTROL_TASK_STACK_SIZE,    // Stack size
        NULL,                       // Parameters
        CONTROL_TASK_PRIORITY,      // Priority
        &controlTaskHandle,         // Task handle
        1                           // Core (1 = Arduino core)
    );

    if (xReturned != pdPASS) {
        handleSystemError("Failed to create ControlTask", true);
        return; // This line is never reached after restart
    }
    
    BaseType_t xReturnedControl = xTaskCreatePinnedToCore(
        WebSocketTask,
        "WebSocketTask",
        WEBSOCKET_TASK_STACK_SIZE,
        NULL,
        WEBSOCKET_TASK_PRIORITY,
        &websocketTaskHandle,
        0
    );

    if (xReturnedControl != pdPASS) {
        handleSystemError("Failed to create WebSocketTask", true);
        return;
    }
    
    // Create HTTP server task
    BaseType_t xReturnedHttp = xTaskCreatePinnedToCore(
        HttpServerTask,
        "HttpServerTask",
        HTTP_SERVER_TASK_STACK_SIZE,
        NULL,
        HTTP_SERVER_TASK_PRIORITY,
        &httpServerTaskHandle,
        0  // Run on core 0, leaving core 1 for control tasks
    );

    if (xReturnedHttp != pdPASS) {
        handleSystemError("Failed to create HttpServerTask", true);
        return;
    }
    
    // Create GNSS task
    BaseType_t xReturnedGNSS = xTaskCreatePinnedToCore(
        GNSSTask,
        "GNSSTask",
        GNSS_TASK_STACK_SIZE,
        NULL,
        GNSS_TASK_PRIORITY,
        &gnssTaskHandle,
        1  // Run on core 1 for time-sensitive operations
    );

    if (xReturnedGNSS != pdPASS) {
        handleSystemError("Failed to create GNSSTask", true);
        return;
    }

    // Create the navigation task
    BaseType_t xReturnedNav = xTaskCreatePinnedToCore(
        NavigationTask,
        "NavTask",
        NAV_TASK_STACK_SIZE,
        NULL,
        NAV_TASK_PRIORITY,
        &navTaskHandle,
        1  // Pin to Core 1
    );
    
    if (xReturnedNav != pdPASS) {
        handleSystemError("Failed to create NavigationTask", true);
        return;
    }

    // Create the logging task
    BaseType_t xReturnedLog = xTaskCreatePinnedToCore(
        logTask,
        "LogTask",
        LOG_TASK_STACK_SIZE,
        NULL,
        LOG_TASK_PRIORITY,
        &logTaskHandle,
        0 // Run on core 0
    );

    if (xReturnedLog != pdPASS) {
        handleSystemError("Failed to create LogTask", true);
        return;
    }

    Serial.println("RTOS tasks created");
    LOG_DEBUG("RTOS tasks created");
}

// Add to a utilities or system header file
void handleSystemError(const char* errorMsg, bool restartSystem) {
    LOG_ERROR("SYSTEM ERROR: %s", errorMsg);
    
    // Clean up all resources
    cleanupResources(true, true, true);
    
    if (restartSystem) {
        LOG_ERROR("System will restart in 3 seconds");
        vTaskDelay(pdMS_TO_TICKS(3000));
        ESP.restart();
    }
}

void cleanupResources(bool cleanupMutexes, bool cleanupQueues, bool cleanupTasks) {
    // Clean up tasks if requested
    if (cleanupTasks) {
        if (controlTaskHandle != NULL) {
            vTaskDelete(controlTaskHandle);
            controlTaskHandle = NULL;
            LOG_DEBUG("ControlTask deleted");
        }

        if (navTaskHandle != NULL) {
            vTaskDelete(navTaskHandle);
            navTaskHandle = NULL;
            LOG_DEBUG("NavigationTask deleted");
        }
        
        if (gnssTaskHandle != NULL) {
            vTaskDelete(gnssTaskHandle);
            gnssTaskHandle = NULL;
            LOG_DEBUG("GNSSTask deleted");
        }

        if (websocketTaskHandle != NULL) {
            vTaskDelete(websocketTaskHandle);
            websocketTaskHandle = NULL;
            LOG_DEBUG("WebSocketTask deleted");
        }

        if (httpServerTaskHandle != NULL) {
            vTaskDelete(httpServerTaskHandle);
            httpServerTaskHandle = NULL;
            LOG_DEBUG("httpServerTask deleted");
        }

        if (logTaskHandle != NULL) {
            vTaskDelete(logTaskHandle);
            httpServerTaskHandle = NULL;
            LOG_DEBUG("logTask deleted");
        }
    }
    
    // Clean up queues if requested
    if (cleanupQueues) {
        if (commandQueue != NULL) {
            vQueueDelete(commandQueue);
            commandQueue = NULL;
            LOG_DEBUG("Command queue deleted");
        }
        
        if (navCommandQueue != NULL) {
            vQueueDelete(navCommandQueue);
            navCommandQueue = NULL;
            LOG_DEBUG("Nav command queue deleted");
        }
        
        if (logQueue != NULL) {
            vQueueDelete(logQueue);
            logQueue = NULL;
            LOG_DEBUG("Log queue deleted");
        }
    }
    
    // Clean up mutexes if requested
    if (cleanupMutexes) {
        if (servoMutex != NULL) {
            vSemaphoreDelete(servoMutex);
            servoMutex = NULL;
            LOG_DEBUG("Servo mutex deleted");
        }
        
        if (gnssMutex != NULL) {
            vSemaphoreDelete(gnssMutex);
            gnssMutex = NULL;
            LOG_DEBUG("GNSS mutex deleted");
        }
        
        if (ntripClientMutex != NULL) {
            vSemaphoreDelete(ntripClientMutex);
            ntripClientMutex = NULL;
            LOG_DEBUG("NTRIP client mutex deleted");
        }
        
        if (navDataMutex != NULL) {
            vSemaphoreDelete(navDataMutex);
            navDataMutex = NULL;
            LOG_DEBUG("Nav data mutex deleted");
        }
        
        if (waypointMutex != NULL) {
            vSemaphoreDelete(waypointMutex);
            waypointMutex = NULL;
            LOG_DEBUG("Waypoint mutex deleted");
        }
        
        if (logFileMutex != NULL) {
            vSemaphoreDelete(logFileMutex);
            logFileMutex = NULL;
            LOG_DEBUG("Log file mutex deleted");
        }
    }
}