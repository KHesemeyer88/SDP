#include "logging.h"
#include <stdio.h>
#include <stdarg.h>

// Log file definitions
#define LOG_FILE_PATH "/log.txt"
#define LOG_QUEUE_SIZE 50
#define LOG_TASK_STACK_SIZE 4096
#define LOG_TASK_PRIORITY 2
#define LOG_FLUSH_INTERVAL 5000  // 5 seconds

// Global variables
bool loggingEnabled = true;
QueueHandle_t logQueue = NULL;
TaskHandle_t logTaskHandle = NULL;
static File logFile;
static SemaphoreHandle_t logFileMutex = NULL;
static const char* logLevelNames[] = {"DEBUG", "INFO", "WARNING", "ERROR"};

// Initialize the logging system
bool initLogging() {
    if (!loggingEnabled) return true; // Successfully "disabled"

    Serial.println("Initializing SD card for logging...");
    
    // Configure SPI for SD card
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    
    // Initialize SD card
    if (!SD.begin(SD_CS_PIN, SPI, SD_FREQUENCY)) {
        Serial.println("SD Card initialization failed. Logging disabled.");
        loggingEnabled = false;
        return false;
    }
    
    Serial.println("SD Card initialized successfully for logging.");
    
    // Create mutex for file access
    logFileMutex = xSemaphoreCreateMutex();
    if (logFileMutex == NULL) {
        Serial.println("Failed to create log file mutex!");
        loggingEnabled = false;
        return false;
    }
    
    // Create log message queue
    logQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(LogMessage));
    if (logQueue == NULL) {
        Serial.println("Failed to create log queue!");
        loggingEnabled = false;
        return false;
    }
    
    // Create the logging task
    BaseType_t xReturned = xTaskCreatePinnedToCore(
        logTask,
        "LogTask",
        LOG_TASK_STACK_SIZE,
        NULL,
        LOG_TASK_PRIORITY,
        &logTaskHandle,
        0 // Run on core 0
    );
    
    if (xReturned != pdPASS) {
        Serial.println("Failed to create log task!");
        loggingEnabled = false;
        return false;
    }
    
    // Open the log file
    if (xSemaphoreTake(logFileMutex, portMAX_DELAY) == pdTRUE) {
        logFile = SD.open(LOG_FILE_PATH, FILE_APPEND);
        if (!logFile) {
            Serial.println("Failed to open log file!");
            xSemaphoreGive(logFileMutex);
            loggingEnabled = false;
            return false;
        }
        
        // Write log header
        logFile.println("\n----- New Logging Session Started -----");
        logFile.println("Timestamp,Level,Task,Message");
        logFile.flush();
        
        xSemaphoreGive(logFileMutex);
    } else {
        Serial.println("Failed to obtain log file mutex!");
        loggingEnabled = false;
        return false;
    }
    
    // Log the initialization event
    LOG_INFO("Logging system initialized successfully");
    return true;
}

// The logging task function
void logTask(void *pvParameters) {
    LogMessage msg;
    unsigned long lastFlushTime = millis();
    
    Serial.println("Log task started");
    
    for (;;) {
        // Check if there are messages in the queue
        if (xQueueReceive(logQueue, &msg, pdMS_TO_TICKS(100)) == pdPASS) {
            // Take mutex before writing to file
            if (xSemaphoreTake(logFileMutex, portMAX_DELAY) == pdTRUE) {
                // Format: timestamp,level,task,message
                logFile.printf("%u,%s,%s,%s\n", 
                    msg.timestamp,
                    logLevelNames[msg.level],
                    msg.taskName,
                    msg.message);
                
                // Check if it's time to flush
                unsigned long currentTime = millis();
                if (currentTime - lastFlushTime >= LOG_FLUSH_INTERVAL) {
                    logFile.flush();
                    lastFlushTime = currentTime;
                }
                
                xSemaphoreGive(logFileMutex);
            }
        } else {
            // No message received, check if we should flush
            unsigned long currentTime = millis();
            if (currentTime - lastFlushTime >= LOG_FLUSH_INTERVAL) {
                if (xSemaphoreTake(logFileMutex, portMAX_DELAY) == pdTRUE) {
                    logFile.flush();
                    lastFlushTime = currentTime;
                    xSemaphoreGive(logFileMutex);
                }
            }
        }
        
        // Small delay to prevent excessive CPU usage when queue is empty
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Log a message with the given level
bool logMessage(LogLevel level, const char* format, ...) {
    if (!loggingEnabled || logQueue == NULL) return false;
    
    LogMessage msg;
    msg.level = level;
    msg.timestamp = millis();
    
    // Get current task name
    TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
    if (currentTask != NULL) {
        strlcpy(msg.taskName, pcTaskGetTaskName(currentTask), sizeof(msg.taskName));
    } else {
        strlcpy(msg.taskName, "Unknown", sizeof(msg.taskName));
    }
    
    // Format the message string
    va_list args;
    va_start(args, format);
    vsnprintf(msg.message, sizeof(msg.message), format, args);
    va_end(args);
    
    // Also print to Serial for debug purposes
    Serial.printf("[%u][%s][%s] %s\n", msg.timestamp, logLevelNames[msg.level], msg.taskName, msg.message);
    
    // Send message to queue with timeout to prevent blocking
    return (xQueueSend(logQueue, &msg, pdMS_TO_TICKS(100)) == pdPASS);
}

// Close the logging system
void closeLogging() {
    if (!loggingEnabled) return;
    
    // Log the closing event
    LOG_INFO("Logging system shutting down");
    
    // Wait for queue to empty (with timeout)
    unsigned long startTime = millis();
    while (uxQueueMessagesWaiting(logQueue) > 0 && (millis() - startTime < 1000)) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Close the file
    if (xSemaphoreTake(logFileMutex, portMAX_DELAY) == pdTRUE) {
        logFile.println("----- Logging Session Ended -----");
        logFile.flush();
        logFile.close();
        xSemaphoreGive(logFileMutex);
    }
    
    // Delete the task if it exists
    if (logTaskHandle != NULL) {
        vTaskDelete(logTaskHandle);
        logTaskHandle = NULL;
    }
    
    // Clean up queue and mutex
    if (logQueue != NULL) {
        vQueueDelete(logQueue);
        logQueue = NULL;
    }
    
    if (logFileMutex != NULL) {
        vSemaphoreDelete(logFileMutex);
        logFileMutex = NULL;
    }
    
    loggingEnabled = false;
}