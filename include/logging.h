#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "config.h"

enum LogLevel {
    LOG_NONE,       // No logging at all
    LOG_ERROR,      // Errors and warnings (issues that need attention)
    LOG_DEBUG,      // General information and debug details
    LOG_PERF        // Performance metrics and timing data
};

extern LogLevel currentLogLevel;

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
#define LOG_ERROR(fmt, ...) logMessage(LOG_ERROR, fmt, ##__VA_ARGS__)
#define LOG_PERF(fmt, ...) logMessage(LOG_PERF, fmt, ##__VA_ARGS__)

#endif // LOGGING_H