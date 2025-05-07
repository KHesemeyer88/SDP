#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "config.h"
#include <vector>

enum LogLevel {
    LOG_NONE,       // No logging at all
    LOG_ERROR,      // Errors and warnings (issues that need attention)
    LOG_NAV_STATE,  // Navigation state transitions and commands (debugging auto nav)
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
extern SemaphoreHandle_t logFileMutex;

// Function declarations
bool initLogging();
void logTask(void *pvParameters);
bool logMessage(LogLevel level, const char* format, ...);
void closeLogging();

// Helper macros for easier logging
#define LOG_DEBUG(fmt, ...) logMessage(LOG_DEBUG, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) logMessage(LOG_ERROR, fmt, ##__VA_ARGS__)
#define LOG_PERF(fmt, ...) logMessage(LOG_PERF, fmt, ##__VA_ARGS__)
#define LOG_NAV(fmt, ...) logMessage(LOG_NAV_STATE, fmt, ##__VA_ARGS__)

const char* logLevelToString(LogLevel level);

// Save a waypoint to a named route file on SD
bool saveWaypointToNamedRoute(const char* routeName, float lat, float lon, int rtkStatus, int fixType);
// Get a list of route file names from the SD card
std::vector<String> getRouteFileNames();
// Load waypoints from a named route file on SD
int loadRouteWaypoints(float* lats, float* lons, int maxPoints, const char* routeName);


#endif // LOGGING_H