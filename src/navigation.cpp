// navigation.cpp
#include "navigation.h"
#include "gnss.h"
#include "logging.h"
#include "rtos_tasks.h"
#include "config.h"
#include <Arduino.h>

// Task handle
TaskHandle_t navTaskHandle = NULL;

// Mutexes for protecting shared data
SemaphoreHandle_t navDataMutex = NULL;
SemaphoreHandle_t waypointMutex = NULL;

// Queue for navigation commands
QueueHandle_t navCommandQueue = NULL;

// Waypoints array
Waypoint waypoints[MAX_WAYPOINTS];
int waypointCount = 0;

// Shared navigation status and target data
volatile NavStatus navStatus;
volatile TargetData targetData;
extern volatile GNSSData gnssData;

// Internal helper functions (not exposed in header)
static void processNavigationCommand(NavCommand cmd);
static void updateNavigationStatus(float lat, float lon, float speed, uint8_t fixType);
static void checkDestinationStatus(float lat, float lon);
static void updateTargetData();
static void handleWaypointReached();
static void initNavStatusStruct(volatile NavStatus* status);

// Define a specific initialization function for NavStatus
void initNavStatusStruct(volatile NavStatus* status) {
    if (status) {
        status->autonomousMode = false;
        status->isPaused = false;
        status->distanceTraveled = 0.0;
        status->currentPace = 0.0;
        status->averagePace = 0.0;
        status->elapsedTime = 0;
        status->distanceToWaypoint = 0.0;
        status->targetPace = DEFAULT_TARGET_PACE;
        status->targetDistance = DEFAULT_TARGET_DISTANCE;
        status->currentWaypoint = 0;
        status->totalWaypoints = 0;
        // Cast away volatile to initialize and then restore it
        String* nonVolatileMsg = const_cast<String*>(&(status->statusMessage));
        *nonVolatileMsg = ""; // Now we can assign to the non-volatile pointer
        status->destinationReached = false;
    }
}

// Update the initialization function
bool initNavigation() {
    LOG_DEBUG("Initializing navigation system");
    
    // Create mutexes for thread-safe access to navigation data
    navDataMutex = xSemaphoreCreateMutex();
    if (navDataMutex == NULL) {
        LOG_ERROR("Failed to create navigation data mutex");
        return false;
    }
    
    waypointMutex = xSemaphoreCreateMutex();
    if (waypointMutex == NULL) {
        LOG_ERROR("Failed to create waypoint mutex");
        return false;
    }
    
    // Create command queue
    navCommandQueue = xQueueCreate(5, sizeof(NavCommand));
    if (navCommandQueue == NULL) {
        LOG_ERROR("Failed to create navigation command queue");
        return false;
    }
    
    // Initialize navigation status
    initNavStatusStruct(&navStatus);
    
    // Initialize target data
    targetData.targetLat = 0.0;
    targetData.targetLon = 0.0;
    targetData.followingWaypoints = false;
    
    // Initialize waypoints
    waypointCount = 0;
    
    // Create the navigation task
    BaseType_t result = xTaskCreate(
        NavigationTask,
        "NavTask",
        NAV_TASK_STACK_SIZE,
        NULL,
        NAV_TASK_PRIORITY,
        &navTaskHandle
    );
    
    if (result != pdPASS) {
        LOG_ERROR("Failed to create navigation task");
        return false;
    }
    
    LOG_DEBUG("Navigation system initialized");
    return true;
}

// Start navigation to a single waypoint
bool startNavigation(float targetPace, float targetDistance, float lat, float lon) {
    LOG_DEBUG("Starting navigation to %f, %f with pace %f m/s", lat, lon, targetPace);
    
    // Create and send command to navigation task
    NavCommand cmd;
    cmd.type = NAV_CMD_START;
    cmd.start.targetPace = targetPace;
    cmd.start.targetDistance = targetDistance;
    cmd.start.latitude = lat;
    cmd.start.longitude = lon;
    
    // Send the command to the navigation task queue
    if (xQueueSend(navCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        LOG_ERROR("Failed to send start navigation command");
        return false;
    }
    
    return true;
}

// Start navigation using pre-recorded waypoints
bool startWaypointNavigation(float targetPace, float targetDistance) {
    LOG_DEBUG("Starting waypoint navigation with pace %f m/s", targetPace);
    
    // Make sure we have waypoints
    if (waypointCount == 0) {
        LOG_ERROR("Cannot start waypoint navigation: no waypoints defined");
        return false;
    }
    
    // Create and send command to navigation task
    NavCommand cmd;
    cmd.type = NAV_CMD_START;
    cmd.start.targetPace = targetPace;
    cmd.start.targetDistance = targetDistance;
    
    // For waypoint navigation, we set the first waypoint as target
    if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        cmd.start.latitude = waypoints[0].latitude;
        cmd.start.longitude = waypoints[0].longitude;
        xSemaphoreGive(waypointMutex);
    } else {
        LOG_ERROR("Failed to access waypoints to start navigation");
        return false;
    }
    
    // Send the command to the navigation task queue
    if (xQueueSend(navCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        LOG_ERROR("Failed to send start waypoint navigation command");
        return false;
    }
    
    // Update target data to use waypoints
    if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        targetData.followingWaypoints = true;
        xSemaphoreGive(waypointMutex);
    }
    
    return true;
}

// Stop navigation
bool stopNavigation() {
    LOG_DEBUG("Stopping navigation");
    
    NavCommand cmd;
    cmd.type = NAV_CMD_STOP;
    
    if (xQueueSend(navCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        LOG_ERROR("Failed to send stop navigation command");
        return false;
    }
    
    return true;
}

// Pause navigation
bool pauseNavigation() {
    LOG_DEBUG("Pausing navigation");
    
    NavCommand cmd;
    cmd.type = NAV_CMD_PAUSE;
    
    if (xQueueSend(navCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        LOG_ERROR("Failed to send pause navigation command");
        return false;
    }
    
    return true;
}

// Resume navigation
bool resumeNavigation() {
    LOG_DEBUG("Resuming navigation");
    
    NavCommand cmd;
    cmd.type = NAV_CMD_RESUME;
    
    if (xQueueSend(navCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        LOG_ERROR("Failed to send resume navigation command");
        return false;
    }
    
    return true;
}

// Reset navigation statistics
bool resetNavigationStats() {
    LOG_DEBUG("Resetting navigation statistics");
    
    NavCommand cmd;
    cmd.type = NAV_CMD_RESET_STATS;
    
    if (xQueueSend(navCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        LOG_ERROR("Failed to send reset stats command");
        return false;
    }
    
    return true;
}

// Navigation task implementation
void NavigationTask(void *pvParameters) {
    // Initialize local variables
    NavCommand cmd;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / NAV_UPDATE_FREQUENCY);
    
    // Local vars for GNSS data
    float currentLat, currentLon, currentSpeed;
    uint8_t fixType;
    int carrSoln;
    double hAcc;
    bool validPosition = false;
    
    // Initialize time for consistent frequency
    xLastWakeTime = xTaskGetTickCount();
    
    LOG_DEBUG("Navigation task started");
    
    // Task loop
    while (true) {
        // Process any pending commands
        while (xQueueReceive(navCommandQueue, &cmd, 0) == pdTRUE) {
            processNavigationCommand(cmd);
        }
        
        // Check if navigation is active and not paused
        bool isActive = false;
        bool isPaused = false;
        
        // Safely get navigation state
        if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            isActive = navStatus.autonomousMode;
            isPaused = navStatus.isPaused;
            xSemaphoreGive(navDataMutex);
        }
        
        // Only process navigation if active and not paused
        if (isActive && !isPaused) {
            // Get current position data
            validPosition = false;
            
            // Try to get GNSS data with timeout
            if (xSemaphoreTake(gnssMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                currentLat = gnssData.latitude;
                currentLon = gnssData.longitude;
                currentSpeed = gnssData.speed;
                fixType = gnssData.fixType;
                carrSoln = gnssData.carrSoln;
                hAcc = gnssData.hAcc;
                
                validPosition = (fixType >= 3);
                xSemaphoreGive(gnssMutex);
            }
            
            if (validPosition) {
                // Update navigation with the copied values
                updateNavigationStatus(currentLat, currentLon, currentSpeed, fixType);
                
                // Check if target reached
                checkDestinationStatus(currentLat, currentLon);
                
                // Update target data
                updateTargetData();
            }
        }
        
        // Delay until next cycle for consistent timing
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

static void processNavigationCommand(NavCommand cmd) {
    LOG_DEBUG("Processing navigation command type: %d", cmd.type);
    
    switch (cmd.type) {
        case NAV_CMD_START:
            // Start navigation to a single waypoint
            if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                navStatus.autonomousMode = true;
                navStatus.isPaused = false;
                navStatus.targetPace = cmd.start.targetPace;
                navStatus.targetDistance = cmd.start.targetDistance;
                navStatus.distanceTraveled = 0.0;
                navStatus.elapsedTime = 0;
                navStatus.currentPace = 0.0;
                navStatus.averagePace = 0.0;
                xSemaphoreGive(navDataMutex);
            }
            
            if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                // Set target to provided coordinates
                targetData.targetLat = cmd.start.latitude;
                targetData.targetLon = cmd.start.longitude;
                targetData.followingWaypoints = false;
                xSemaphoreGive(waypointMutex);
            }
            break;
            
        case NAV_CMD_STOP:
            // Stop navigation
            if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                navStatus.autonomousMode = false;
                navStatus.isPaused = false;
                
                // Set status message
                String* nonVolatileMsg = const_cast<String*>(&(navStatus.statusMessage));
                *nonVolatileMsg = "Navigation stopped";
                
                xSemaphoreGive(navDataMutex);
            }
            break;
            
        case NAV_CMD_PAUSE:
            // Pause navigation without losing progress
            if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                navStatus.isPaused = true;
                
                // Set status message
                String* nonVolatileMsg = const_cast<String*>(&(navStatus.statusMessage));
                *nonVolatileMsg = "Navigation paused";
                
                xSemaphoreGive(navDataMutex);
            }
            break;
            
        case NAV_CMD_RESUME:
            // Resume navigation from paused state
            if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (navStatus.autonomousMode) {
                    navStatus.isPaused = false;
                    
                    // Set status message
                    String* nonVolatileMsg = const_cast<String*>(&(navStatus.statusMessage));
                    *nonVolatileMsg = "Navigation resumed";
                }
                xSemaphoreGive(navDataMutex);
            }
            break;
            
        case NAV_CMD_RESET_STATS:
            // Reset distance and time tracking
            if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                navStatus.distanceTraveled = 0.0;
                navStatus.elapsedTime = 0;
                navStatus.currentPace = 0.0;
                navStatus.averagePace = 0.0;
                xSemaphoreGive(navDataMutex);
            }
            break;
            
        case NAV_CMD_ADD_WAYPOINT:
            // Add a waypoint to the list
            if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (waypointCount < MAX_WAYPOINTS) {
                    waypoints[waypointCount].latitude = cmd.waypoint.latitude;
                    waypoints[waypointCount].longitude = cmd.waypoint.longitude;
                    waypointCount++;
                    
                    if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        navStatus.totalWaypoints = waypointCount;
                        xSemaphoreGive(navDataMutex);
                    }
                }
                xSemaphoreGive(waypointMutex);
            }
            break;
            
        case NAV_CMD_CLEAR_WAYPOINTS:
            // Clear all waypoints
            if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                waypointCount = 0;
                
                if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    navStatus.totalWaypoints = 0;
                    navStatus.currentWaypoint = 0;
                    xSemaphoreGive(navDataMutex);
                }
                
                xSemaphoreGive(waypointMutex);
            }
            break;
            
        // Other commands could be implemented here
        
        default:
            LOG_ERROR("Unknown navigation command type: %d", cmd.type);
            break;
    }
}

static void updateNavigationStatus(float lat, float lon, float speed, uint8_t fixType) {
    static unsigned long lastUpdateTime = 0;
    static float lastPositionLat = 0.0;
    static float lastPositionLon = 0.0;
    unsigned long currentTime = millis();
    
    // Only update if we have valid previous coordinates
    if (lastPositionLat != 0 && lastPositionLon != 0) {
        float delta = calculateDistance(lastPositionLat, lastPositionLon, lat, lon);
        
        // Filter out unreasonable jumps (GPS errors)
        if (delta < 10.0 && speed > 0.5) {
            // Update distance traveled
            if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                navStatus.distanceTraveled += delta;
                navStatus.currentPace = speed;
                
                // Calculate elapsed time and average pace
                unsigned long timeDelta = currentTime - lastUpdateTime;
                navStatus.elapsedTime += timeDelta;
                
                if (navStatus.elapsedTime > 0) {
                    navStatus.averagePace = (navStatus.distanceTraveled / 
                                           (navStatus.elapsedTime / 1000.0));
                }
                
                xSemaphoreGive(navDataMutex);
            }
        }
    }
    
    // Update for next calculation
    lastPositionLat = lat;
    lastPositionLon = lon;
    lastUpdateTime = currentTime;
    
    // Also update distance to waypoint
    if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        float targetLat = targetData.targetLat;
        float targetLon = targetData.targetLon;
        xSemaphoreGive(waypointMutex);
        
        float distToWaypoint = calculateDistance(lat, lon, targetLat, targetLon);
        
        if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            navStatus.distanceToWaypoint = distToWaypoint;
            xSemaphoreGive(navDataMutex);
        }
    }
}

// Check if we've reached the target waypoint or distance
static void checkDestinationStatus(float lat, float lon) {
    float distToWaypoint = 0.0;
    bool followingWaypoints = false;
    
    // Get target data
    if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        float targetLat = targetData.targetLat;
        float targetLon = targetData.targetLon;
        followingWaypoints = targetData.followingWaypoints;
        xSemaphoreGive(waypointMutex);
        
        // Calculate distance to waypoint
        distToWaypoint = calculateDistance(lat, lon, targetLat, targetLon);
    }
    
    // Check if we've reached the waypoint
    if (distToWaypoint < WAYPOINT_REACHED_RADIUS) {
        LOG_DEBUG("Waypoint reached (distance: %.2f m)", distToWaypoint);
        handleWaypointReached();
    }
    
    // Check if we've reached target distance
    if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        float targetDistance = navStatus.targetDistance;
        float distanceTraveled = navStatus.distanceTraveled;
        
        if (targetDistance > 0 && distanceTraveled >= targetDistance) {
            // Set status to indicate target distance reached
            navStatus.autonomousMode = false;
            navStatus.isPaused = false;
            navStatus.destinationReached = true;
            
            // Update status message
            String* nonVolatileMsg = const_cast<String*>(&(navStatus.statusMessage));
            *nonVolatileMsg = "Target distance reached";
            
            LOG_DEBUG("Target distance of %.2f m reached (traveled: %.2f m)", 
                     targetDistance, distanceTraveled);
        }
        
        xSemaphoreGive(navDataMutex);
    }
}

// Handle waypoint reached - move to next waypoint or stop
static void handleWaypointReached() {
    bool followingWaypoints = false;
    int currentWaypoint = 0;
    int totalWaypoints = 0;
    
    // Get current waypoint status
    if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        currentWaypoint = navStatus.currentWaypoint;
        xSemaphoreGive(navDataMutex);
    }
    
    // Get total waypoints and following status
    if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        followingWaypoints = targetData.followingWaypoints;
        totalWaypoints = waypointCount;
        xSemaphoreGive(waypointMutex);
    }
    
    // Handle waypoint reaching based on mode
    if (followingWaypoints) {
        // Following multiple waypoints
        if (currentWaypoint < totalWaypoints - 1) {
            // Move to next waypoint
            currentWaypoint++;
            
            // Update current waypoint in status
            if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                navStatus.currentWaypoint = currentWaypoint;
                
                // Update status message
                String* nonVolatileMsg = const_cast<String*>(&(navStatus.statusMessage));
                *nonVolatileMsg = "Moving to next waypoint";
                
                xSemaphoreGive(navDataMutex);
            }
            
            // Update target to new waypoint
            if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (currentWaypoint < waypointCount) {
                    targetData.targetLat = waypoints[currentWaypoint].latitude;
                    targetData.targetLon = waypoints[currentWaypoint].longitude;
                }
                xSemaphoreGive(waypointMutex);
            }
            
            LOG_DEBUG("Moving to waypoint %d of %d", currentWaypoint + 1, totalWaypoints);
        } else {
            // Last waypoint reached - either loop back or stop
            // For now, let's implement loop back to first waypoint
            
            // Reset to first waypoint
            currentWaypoint = 0;
            
            // Update current waypoint in status
            if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                navStatus.currentWaypoint = currentWaypoint;
                
                // Update status message
                String* nonVolatileMsg = const_cast<String*>(&(navStatus.statusMessage));
                *nonVolatileMsg = "Starting waypoint sequence again";
                
                xSemaphoreGive(navDataMutex);
            }
            
            // Update target to first waypoint
            if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (waypointCount > 0) {
                    targetData.targetLat = waypoints[0].latitude;
                    targetData.targetLon = waypoints[0].longitude;
                }
                xSemaphoreGive(waypointMutex);
            }
            
            LOG_DEBUG("Last waypoint reached, restarting from waypoint 1");
        }
    } else {
        // Single waypoint mode - destination reached
        if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            navStatus.autonomousMode = false;
            navStatus.isPaused = false;
            navStatus.destinationReached = true;
            
            // Update status message
            String* nonVolatileMsg = const_cast<String*>(&(navStatus.statusMessage));
            *nonVolatileMsg = "Destination reached";
            
            xSemaphoreGive(navDataMutex);
        }
        
        LOG_DEBUG("Single destination reached, stopping navigation");
    }
}

// Update target data shared with control task
static void updateTargetData() {
    // Mostly just to notify control task if anything needs updating
    // This is a lightweight function since we've already done most 
    // updates in other functions as needed
    
    // Potentially could add notification mechanism here
    // to trigger immediate control task response to new targets
}

// Get current navigation status (thread-safe)
NavStatus getNavStatus() {
    NavStatus status;
    
    if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Copy the volatile status to local variable
        memcpy(&status, (const void*)&navStatus, sizeof(NavStatus));
        xSemaphoreGive(navDataMutex);
    } else {
        // If mutex can't be taken, return empty status
        memset(&status, 0, sizeof(NavStatus));
    }
    
    return status;
}

// Add a waypoint to the list (thread-safe)
bool addWaypoint(float latitude, float longitude) {
    NavCommand cmd;
    cmd.type = NAV_CMD_ADD_WAYPOINT;
    cmd.waypoint.latitude = latitude;
    cmd.waypoint.longitude = longitude;
    
    if (xQueueSend(navCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        LOG_ERROR("Failed to send add waypoint command");
        return false;
    }
    
    return true;
}

// Clear all waypoints (thread-safe)
bool clearWaypoints() {
    NavCommand cmd;
    cmd.type = NAV_CMD_CLEAR_WAYPOINTS;
    
    if (xQueueSend(navCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        LOG_ERROR("Failed to send clear waypoints command");
        return false;
    }
    
    return true;
}

// Get current waypoint count (thread-safe)
int getWaypointCount() {
    int count = 0;
    
    if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        count = waypointCount;
        xSemaphoreGive(waypointMutex);
    }
    
    return count;
}

// Calculate bearing between two coordinates
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    float lat1Rad = lat1 * PI / 180.0;
    float lon1Rad = lon1 * PI / 180.0;
    float lat2Rad = lat2 * PI / 180.0;
    float lon2Rad = lon2 * PI / 180.0;

    float dLon = lon2Rad - lon1Rad;
    float y = sin(dLon) * cos(lat2Rad);
    float x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(dLon);

    float bearing = atan2(y, x) * 180.0 / PI;
    return fmod((bearing + 360.0), 360.0);
}

// Calculate distance between two coordinates using the Haversine formula
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    float lat1Rad = lat1 * PI / 180.0;
    float lon1Rad = lon1 * PI / 180.0;
    float lat2Rad = lat2 * PI / 180.0;
    float lon2Rad = lon2 * PI / 180.0;

    float dLat = lat2Rad - lat1Rad;
    float dLon = lon2Rad - lon1Rad;
    float a = sin(dLat/2) * sin(dLat/2) +
              cos(lat1Rad) * cos(lat2Rad) *
              sin(dLon/2) * sin(dLon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    return c * 6371000; // Earth radius in meters
}