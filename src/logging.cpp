#include "logging.h"
#include <stdio.h>
#include <stdarg.h>

// Log file definitions
#define LOG_FILE_PATH "/log.txt"
#define LOG_QUEUE_SIZE 50
#define LOG_FLUSH_INTERVAL 5000  // 5 seconds
#define ROUTES_DIR "/routes"

// Global variables
LogLevel currentLogLevel = LOG_ERROR; // Default to debug level
QueueHandle_t logQueue = NULL;
TaskHandle_t logTaskHandle = NULL;
static File logFile;
SemaphoreHandle_t logFileMutex = NULL;
static const char* logLevelNames[] = {"NONE", "ERROR", "NAV", "DEBUG", "PERF"};
const char* logLevelToString(LogLevel level) {
    if (level < 0 || level >= (int)(sizeof(logLevelNames)/sizeof(logLevelNames[0]))) {
        return "INVALID";
    }
    return logLevelNames[level];
}


// Initialize the logging system
bool initLogging() {
    Serial.printf("starting initLogging");
    Serial.printf("\n");
    if (currentLogLevel == LOG_NONE) {
        Serial.printf("currentLogLevel set to NONE in script");
        Serial.printf("\n");
        return true; // Logging disabled
    } 

    // fix brownout? no
    delay(1000);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Configure SPI for SD card
    static SPIClass sdSPI(VSPI);
    sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

    // Initialize SD card
    if (!SD.begin(SD_CS_PIN, sdSPI, SD_FREQUENCY)) {
        // Can't use logging here, but we still need some indication
        // Just return false - the calling function can decide what to do
        Serial.printf("SD fail - currentLogLevel being set to NONE");
        Serial.printf("\n");
        currentLogLevel = LOG_NONE;
        return false;
    }
    
    // Create mutex for file access
    logFileMutex = xSemaphoreCreateMutex();
    if (logFileMutex == NULL) {
        Serial.printf("logFileMutex fail - currentLogLevel being set to NONE");
        Serial.printf("\n");
        currentLogLevel = LOG_NONE;
        return false;
    }
    
    // Create log message queue
    Serial.printf("Before xQueueCreate, heap free: %u\n", ESP.getFreeHeap());
    Serial.printf("\n");
    logQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(LogMessage));
    if (logQueue == NULL) {
        Serial.printf("logQueue fail - currentLogLevel being set to NONE");
        Serial.printf("\n");
        currentLogLevel = LOG_NONE;
        return false;
    }
    
    // Open the log file
    if (xSemaphoreTake(logFileMutex, portMAX_DELAY) == pdTRUE) {
        logFile = SD.open(LOG_FILE_PATH, FILE_APPEND);
        if (!logFile) {
            xSemaphoreGive(logFileMutex);
            Serial.printf("xSemaphoreTake of logFileMutex fail - currentLogLevel being set to NONE");
            Serial.printf("\n");
            currentLogLevel = LOG_NONE;
            return false;
        }
        
        // Write log header
        logFile.println("\n----- New Logging Session Started -----");
        logFile.println("Timestamp,Level,Task,Message");
        logFile.flush();
        
        xSemaphoreGive(logFileMutex);
    } else {
        Serial.printf("OTHER fail - currentLogLevel being set to NONE");
        Serial.printf("\n");
        currentLogLevel = LOG_NONE;
        return false;
    }
    
    return true;
}

#define LOG_LEVEL_COUNT 5  // NONE, ERROR, NAV, DEBUG, PERF

void logTask(void *pvParameters) {
    if (logQueue == NULL || logFileMutex == NULL) {
        vTaskDelete(NULL);
        return;
    }

    LogMessage msg;
    unsigned long lastFlushTime = millis();

    while (true) {
        bool wroteLine = false;

        if (xQueueReceive(logQueue, &msg, pdMS_TO_TICKS(100)) == pdPASS) {
            if (msg.level < 0 || msg.level >= LOG_LEVEL_COUNT) {
                continue;
            }

            msg.taskName[sizeof(msg.taskName) - 1] = '\0';
            msg.message[sizeof(msg.message) - 1] = '\0';

            char lineBuf[300];
            snprintf(lineBuf, sizeof(lineBuf), "%u,%s,%s,%s",
                     msg.timestamp,
                     logLevelNames[msg.level],
                     msg.taskName,
                     msg.message);

            if (xSemaphoreTake(logFileMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                if (logFile) {
                    logFile.println(lineBuf);
                    wroteLine = true;
                }
                xSemaphoreGive(logFileMutex);
            }
        }

        // Flush only if needed
        unsigned long now = millis();
        if ((wroteLine || (now - lastFlushTime >= LOG_FLUSH_INTERVAL)) &&
            xSemaphoreTake(logFileMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (logFile) logFile.flush();
            xSemaphoreGive(logFileMutex);
            lastFlushTime = now;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // yield
    }
}

bool logMessage(LogLevel level, const char* format, ...) {
    if (currentLogLevel == LOG_NONE || level > currentLogLevel || logQueue == NULL) return false;
    
    LogMessage msg = {};  // Zero initialize everything

    msg.level = level;
    msg.timestamp = millis();

    // Get task name safely
    TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
    const char* taskName = currentTask ? pcTaskGetTaskName(currentTask) : "Unknown";
    strlcpy(msg.taskName, taskName, sizeof(msg.taskName));

    // Format message string safely
    va_list args;
    va_start(args, format);
    int n = vsnprintf(msg.message, sizeof(msg.message), format, args);
    va_end(args);

    if (n < 0 || n >= (int)sizeof(msg.message)) {
        // Overflow occurred or format error
        strlcpy(msg.message, "[log formatting error]", sizeof(msg.message));
    }

    msg.taskName[sizeof(msg.taskName) - 1] = '\0';
    msg.message[sizeof(msg.message) - 1] = '\0';

    // Send the whole thing safely
    if (xQueueSend(logQueue, &msg, pdMS_TO_TICKS(100)) != pdPASS) {
        // Optionally add drop count tracking here
        return false;
    }

    return true;
}

bool saveWaypointToNamedRoute(const char* routeName, float lat, float lon, int rtkStatus, int fixType) {
    if (!routeName || strlen(routeName) == 0) return false;

    LOG_DEBUG("Saved waypoint to route %s", routeName);
    // Use the same shared SPIClass as initLogging

    // Attempt to lock the SD card access mutex
    if (logFileMutex == nullptr || xSemaphoreTake(logFileMutex, portMAX_DELAY) != pdTRUE) {
        Serial.println("Failed to take SD card mutex in saveWaypointToNamedRoute");
        return false;
    }

    // Ensure routes folder exists
    if (!SD.exists(ROUTES_DIR)) {
        if (!SD.mkdir(ROUTES_DIR)) {
            Serial.println("Failed to create /routes directory");
            return false;
        }
    }

    String filePath = String(ROUTES_DIR) + "/" + String(routeName) + ".csv";

    // If file doesn't exist, create with header
    if (!SD.exists(filePath)) {
        File f = SD.open(filePath, FILE_WRITE);
        if (!f) return false;
        f.println("latitude,longitude,rtk_status,fix_type");
        f.close();
    }

    // Append waypoint
    File f = SD.open(filePath, FILE_APPEND);
    if (!f) return false;

    String line = String(lat, 8) + "," +
                  String(lon, 8) + "," +
                  String(rtkStatus) + "," +
                  String(fixType);
    f.println(line);
    f.close();

    xSemaphoreGive(logFileMutex);

    return true;
}

std::vector<String> getRouteFileNames() {
    std::vector<String> routeNames;

    File dir = SD.open("/routes");
    if (!dir || !dir.isDirectory()) {
        Serial.println("getRouteFileNames: Failed to open /routes directory");
        return routeNames;
    }

    while (true) {
        File entry = dir.openNextFile();
        if (!entry) break;

        if (!entry.isDirectory()) {
            String name = entry.name();  // "/routes/demo.csv"
            if (name.endsWith(".csv")) {
                name.replace("/routes/", "");
                name.replace(".csv", "");
                routeNames.push_back(name);
            }
        }
        entry.close();
    }

    dir.close();
    return routeNames;
}

int loadRouteWaypoints(float* lats, float* lons, int maxPoints, const char* routeName) {
    String path = "/routes/" + String(routeName) + ".csv";
    File f = SD.open(path);
    if (!f) return 0;

    int count = 0;
    while (f.available() && count < maxPoints) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() == 0 || line.startsWith("latitude")) continue;

        int comma1 = line.indexOf(',');
        int comma2 = line.indexOf(',', comma1 + 1);
        if (comma1 == -1 || comma2 == -1) continue;

        lats[count] = line.substring(0, comma1).toFloat();
        lons[count] = line.substring(comma1 + 1, comma2).toFloat();
        count++;
    }
    f.close();
    return count;
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