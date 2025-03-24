#include "logging.h"
#include <stdio.h>
#include <stdarg.h>

// Log file definitions
#define LOG_FILE_PATH "/log.txt"
#define LOG_QUEUE_SIZE 50
#define LOG_FLUSH_INTERVAL 5000  // 5 seconds

// Global variables
LogLevel currentLogLevel = LOG_PERF; // Default to debug level
QueueHandle_t logQueue = NULL;
TaskHandle_t logTaskHandle = NULL;
static File logFile;
SemaphoreHandle_t logFileMutex = NULL;
static const char* logLevelNames[] = {"NONE", "ERROR", "NAV", "DEBUG", "PERF"};

// Initialize the logging system
bool initLogging() {
    if (currentLogLevel == LOG_NONE) return true; // Logging disabled

    // Configure SPI for SD card
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    
    // Initialize SD card
    if (!SD.begin(SD_CS_PIN, SPI, SD_FREQUENCY)) {
        // Can't use logging here, but we still need some indication
        // Just return false - the calling function can decide what to do
        currentLogLevel = LOG_NONE;
        return false;
    }
    
    // Create mutex for file access
    logFileMutex = xSemaphoreCreateMutex();
    if (logFileMutex == NULL) {
        currentLogLevel = LOG_NONE;
        return false;
    }
    
    // Create log message queue
    logQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(LogMessage));
    if (logQueue == NULL) {
        currentLogLevel = LOG_NONE;
        return false;
    }
    
    // Open the log file
    if (xSemaphoreTake(logFileMutex, portMAX_DELAY) == pdTRUE) {
        logFile = SD.open(LOG_FILE_PATH, FILE_APPEND);
        if (!logFile) {
            xSemaphoreGive(logFileMutex);
            currentLogLevel = LOG_NONE;
            return false;
        }
        
        // Write log header
        logFile.println("\n----- New Logging Session Started -----");
        logFile.println("Timestamp,Level,Task,Message");
        logFile.flush();
        
        xSemaphoreGive(logFileMutex);
    } else {
        currentLogLevel = LOG_NONE;
        return false;
    }
    
    return true;
}

// The logging task function
void logTask(void *pvParameters) {
    LogMessage msg;
    unsigned long lastFlushTime = millis();
    
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
    // Skip if logging is disabled (LOG_NONE) or if level is not included in current setting
    if (currentLogLevel == LOG_NONE || level > currentLogLevel || logQueue == NULL) return false;
    
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
    
    // Send message to queue with timeout to prevent blocking
    return (xQueueSend(logQueue, &msg, pdMS_TO_TICKS(100)) == pdPASS);
}

// Close the logging system
void closeLogging() {
    if (currentLogLevel == LOG_NONE) return;
    
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
    
    currentLogLevel = LOG_NONE;
}