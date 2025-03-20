#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "config.h"

// Log levels
enum LogLevel {
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARNING,
    LOG_ERROR
};

// Log message structure
struct LogMessage {
    LogLevel level;
    uint32_t timestamp;
    char taskName[16];
    char message[128];
};

// Global control variables
extern bool loggingEnabled;
extern QueueHandle_t logQueue;
extern TaskHandle_t logTaskHandle;

// Function declarations
bool initLogging();
void logTask(void *pvParameters);
bool logMessage(LogLevel level, const char* format, ...);
void closeLogging();

// Helper macros for easier logging
#define LOG_DEBUG(fmt, ...) logMessage(LOG_DEBUG, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...) logMessage(LOG_INFO, fmt, ##__VA_ARGS__)
#define LOG_WARNING(fmt, ...) logMessage(LOG_WARNING, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) logMessage(LOG_ERROR, fmt, ##__VA_ARGS__)

#endif // LOGGING_H