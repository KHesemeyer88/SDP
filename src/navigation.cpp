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
        memset((void*)status->statusMessage, 0, sizeof(status->statusMessage)); // Clear the message buffer
        status->destinationReached = false;
    }
}

// Update the initialization function
bool initNavigation() {
    //LOG_DEBUG("initNavigation");
    
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
    
    return true;
}

// Start navigation to a single waypoint
bool startNavigation(float targetPace, float targetDistance, float lat, float lon) {
    //LOG_DEBUG("startNavigation, %f, %f, %f, %f", targetDistance, targetPace, lat, lon);
    
    // Create and send command to navigation task
    NavCommand cmd;
    cmd.type = NAV_CMD_START;
    cmd.start.targetPace = targetPace;
    cmd.start.targetDistance = targetDistance;
    cmd.start.latitude = lat;
    cmd.start.longitude = lon;
    
    // Send the command to the navigation task queue
    if (xQueueSend(navCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        LOG_ERROR("navCommandQueue fail");
        return false;
    }
    
    return true;
}

// Start navigation using pre-recorded waypoints
bool startWaypointNavigation(float targetPace, float targetDistance) {
    //LOG_DEBUG("startWaypointNavigation, %f, %f", targetPace, targetDistance);
    
    // Make sure we have waypoints with proper mutex protection
    int wpCount = 0;
    float firstWaypointLat = 0.0;
    float firstWaypointLon = 0.0;
    
    if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        wpCount = waypointCount;
        if (wpCount > 0) {
            firstWaypointLat = waypoints[0].latitude;
            firstWaypointLon = waypoints[0].longitude;
        }
        xSemaphoreGive(waypointMutex);
    }
    
    if (wpCount == 0) {
        LOG_ERROR("wpCount 0");
        return false;
    }
    
    // Create and send command to navigation task
    NavCommand cmd;
    cmd.type = NAV_CMD_START;
    cmd.start.targetPace = targetPace;
    cmd.start.targetDistance = targetDistance;
    cmd.start.latitude = firstWaypointLat;
    cmd.start.longitude = firstWaypointLon;
    
    // Set waypoint navigation flags BEFORE sending command
    if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        targetData.followingWaypoints = true;
        //LOG_NAV("followingWaypoints to true, %d", waypointCount);
        xSemaphoreGive(waypointMutex);
    }
    
    // Reset current waypoint index with proper mutex protection
    if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        navStatus.currentWaypoint = 0;
        navStatus.totalWaypoints = wpCount;
        xSemaphoreGive(navDataMutex);
    }
    
    // Send the command to the navigation task queue
    if (xQueueSend(navCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        LOG_ERROR("navCommandQueue fail in startWaypointNavigation");
        return false;
    }
    
    return true;
}

// Stop navigation
bool stopNavigation() {
    //LOG_DEBUG("stopNavigation");
    
    NavCommand cmd;
    cmd.type = NAV_CMD_STOP;
    
    if (xQueueSend(navCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        LOG_ERROR("navCommandQueue fail in stopNavigation");
        return false;
    }
    
    return true;
}

// Pause navigation
bool pauseNavigation() {
    //LOG_DEBUG("pauseNavigation");
    
    NavCommand cmd;
    cmd.type = NAV_CMD_PAUSE;
    
    if (xQueueSend(navCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        LOG_ERROR("navCommandQueue fail in pauseNavigation");
        return false;
    }
    
    return true;
}

// Resume navigation
bool resumeNavigation() {
    //LOG_DEBUG("resumeNavigation");
    
    NavCommand cmd;
    cmd.type = NAV_CMD_RESUME;
    
    if (xQueueSend(navCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        LOG_ERROR("navCommandQueue fail in resumeNavigation");
        return false;
    }
    
    return true;
}

// Reset navigation statistics
bool resetNavigationStats() {
    //LOG_DEBUG("resetNavigationStats");
    
    NavCommand cmd;
    cmd.type = NAV_CMD_RESET_STATS;
    
    if (xQueueSend(navCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        LOG_ERROR("navCommandQueue fail in resetNavigationStats");
        return false;
    }
    
    return true;
}

// Navigation task implementation
void NavigationTask(void *pvParameters) {
    //LOG_DEBUG("NavigationTask");
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
        
        // Add state tracking
        static bool lastActive = false;
        static bool lastPaused = false;
        if (isActive != lastActive || isPaused != lastPaused) {
            //LOG_NAV("isActive or lastActive chg, %d, %d", isActive, isPaused);
            lastActive = isActive;
            lastPaused = isPaused;
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
    //LOG_NAV("processNavigationCommand, %d", cmd.type);
    
    switch (cmd.type) {
        case NAV_CMD_START:
            //LOG_NAV("NAV_CMD_START, %.2f, %.2f, %d", 
            //    cmd.start.targetPace, cmd.start.targetDistance, targetData.followingWaypoints);
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
                xSemaphoreGive(waypointMutex);
            }
            break;
            
        case NAV_CMD_STOP:
            // LOG_NAV("NAV_CMD_STOP, %d, %d", 
            //     navStatus.autonomousMode, navStatus.isPaused);
            // Stop navigation
            if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                navStatus.autonomousMode = false;
                navStatus.isPaused = false;
                
                // Set status message
                snprintf((char*)navStatus.statusMessage, sizeof(((NavStatus*)0)->statusMessage), "Navigation stopped");
                xSemaphoreGive(navDataMutex);
            }
            break;
            
        case NAV_CMD_PAUSE:
            //LOG_NAV("NAV_CMD_PAUSE");
            // Pause navigation without losing progress
            if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                navStatus.isPaused = true;
                
                // Set status message
                snprintf((char*)navStatus.statusMessage, sizeof(((NavStatus*)0)->statusMessage), "Navigation paused");
                xSemaphoreGive(navDataMutex);
            }
            break;
            
        case NAV_CMD_RESUME:
        
            //LOG_NAV("NAV_CMD_RESUME");
            // Resume navigation from paused state
            if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (navStatus.autonomousMode) {
                    navStatus.isPaused = false;
                    
                    // Set status message
                    snprintf((char*)navStatus.statusMessage, sizeof(((NavStatus*)0)->statusMessage), "Navigation resumed");
                }
                xSemaphoreGive(navDataMutex);
            }
            break;
            
        case NAV_CMD_RESET_STATS:
        //LOG_NAV("NAV_CMD_RESET_STATS");
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
            // LOG_NAV("NAV_CMD_ADD_WAYPOINT, %.7f, %.7f", 
            //     cmd.waypoint.latitude, cmd.waypoint.longitude);

            if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                //LOG_NAV("waypointMutex taken, current count=%d, max=%d", waypointCount, MAX_WAYPOINTS);

                if (waypointCount < MAX_WAYPOINTS) {
                    waypoints[waypointCount].latitude = cmd.waypoint.latitude;
                    waypoints[waypointCount].longitude = cmd.waypoint.longitude;
                    waypointCount++;

                    //LOG_NAV("Waypoint added at index %d, new count=%d", waypointCount-1, waypointCount);
                    
                    if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        navStatus.totalWaypoints = waypointCount;

                        //LOG_NAV("navStatus.totalWaypoints, %d", waypointCount);
                        xSemaphoreGive(navDataMutex);
                    } else {
                        LOG_ERROR("navDataMutex fail in processNavigationCommand");
                    }
                }
                else {
                    LOG_ERROR("max wp count reached");
                }
                xSemaphoreGive(waypointMutex);
                //LOG_NAV("waypointMutex released after ADD_WAYPOINT processing");
            } else {
                LOG_ERROR("waypointMutex fail in processNavigationCommand");
            }
            break;
            
        case NAV_CMD_CLEAR_WAYPOINTS:
        //LOG_NAV("NAV_CMD_CLEAR_WAYPOINTS");
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
    //LOG_DEBUG("updateNavigationStatus");
    static unsigned long lastUpdateTime = 0;
    static float lastPositionLat = 0.0;
    static float lastPositionLon = 0.0;
    unsigned long currentTime = millis();
    
    // Only update if we have valid previous coordinates
    if (lastPositionLat != 0 && lastPositionLon != 0) {
        float delta = calculateDistance(lastPositionLat, lastPositionLon, lat, lon);
        
        // Filter out unreasonable jumps (GPS errors)
        if (delta < 10.0 && speed > 0.5) {
            // Update distance traveled with proper mutex protection
            if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
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
    
    // Also update distance to waypoint with proper mutex protection
    float targetLat = 0.0;
    float targetLon = 0.0;
    
    if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        targetLat = targetData.targetLat;
        targetLon = targetData.targetLon;
        xSemaphoreGive(waypointMutex);
    }
    
    float distToWaypoint = calculateDistance(lat, lon, targetLat, targetLon);
    
    if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        navStatus.distanceToWaypoint = distToWaypoint;
        xSemaphoreGive(navDataMutex);
    }
}

// Check if we've reached the target waypoint or distance
static void checkDestinationStatus(float lat, float lon) {
    //LOG_DEBUG("checkDestinationStatus");
    float distToWaypoint = 0.0;
    bool followingWaypoints = false;
    float targetLat = 0.0;
    float targetLon = 0.0;
    float targetDistance = 0.0;
    float distanceTraveled = 0.0;
    
    // Get all necessary data together with proper mutex protection
    if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        targetLat = targetData.targetLat;
        targetLon = targetData.targetLon;
        followingWaypoints = targetData.followingWaypoints;
        
        if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            targetDistance = navStatus.targetDistance;
            distanceTraveled = navStatus.distanceTraveled;
            
            // Release nav data mutex, no longer needed
            xSemaphoreGive(navDataMutex);
        }
        
        // Release waypoint mutex, no longer needed
        xSemaphoreGive(waypointMutex);
    }
    
    // Calculate distance to waypoint now that we have all the data
    distToWaypoint = calculateDistance(lat, lon, targetLat, targetLon);
    
    // Check if we've reached the waypoint
    if (distToWaypoint < WAYPOINT_REACHED_RADIUS) {
        //LOG_NAV("wp reached, %.2f", distToWaypoint);
        handleWaypointReached();
    }
    
    // Check if we've reached target distance - this needs mutex protection
    if (targetDistance > 0 && distanceTraveled >= targetDistance) {
        // Need to take mutex again to update status
        if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Set status to indicate target distance reached
            navStatus.autonomousMode = false;
            navStatus.isPaused = false;
            navStatus.destinationReached = true;
            
            // Update status message
            snprintf((char*)navStatus.statusMessage, sizeof(((NavStatus*)0)->statusMessage), "Target distance reached");
            
            // LOG_NAV("tgt distance reached, %.2f, %.2f", 
            //          targetDistance, distanceTraveled);
                     
            xSemaphoreGive(navDataMutex);
        }
        
        // Also need to update waypoint following status
        if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            targetData.followingWaypoints = false;
            xSemaphoreGive(waypointMutex);
        }
    }
}

// Handle waypoint reached - move to next waypoint or stop
static void handleWaypointReached() {
    //LOG_DEBUG("handleWaypointReached");
    bool followingWaypoints = false;
    int currentWaypoint = 0;
    int totalWaypoints = 0;
    
    // First safe retrieval for logging purposes
    if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        followingWaypoints = targetData.followingWaypoints;
        totalWaypoints = waypointCount;
        
        if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            currentWaypoint = navStatus.currentWaypoint;
            
            // Important diagnostic logging
            // LOG_NAV("currentWaypoint, totalWaypoints, followingWaypoints, %d, %d, %d", 
            //       currentWaypoint, totalWaypoints, followingWaypoints);
                       
            // Handle waypoint reaching based on mode
            if (followingWaypoints) {
                // Following multiple waypoints
                if (currentWaypoint < totalWaypoints - 1) {
                    // Move to next waypoint
                    currentWaypoint++;
                    navStatus.currentWaypoint = currentWaypoint;
                    
                    // Update target to new waypoint
                    targetData.targetLat = waypoints[currentWaypoint].latitude;
                    targetData.targetLon = waypoints[currentWaypoint].longitude;
                    
                    // Explicitly maintain the waypoint following flag
                    targetData.followingWaypoints = true;
                    
                    // Update status message
                    snprintf((char*)navStatus.statusMessage, sizeof(((NavStatus*)0)->statusMessage), "Moving to next waypoint");

                    //LOG_NAV("Moving to next waypoint: %d of %d", currentWaypoint + 1, totalWaypoints);
                } else {
                    // Reset to first waypoint
                    currentWaypoint = 0;
                    navStatus.currentWaypoint = currentWaypoint;
                    
                    // Update target to first waypoint
                    targetData.targetLat = waypoints[0].latitude;
                    targetData.targetLon = waypoints[0].longitude;
                    
                    // Explicitly maintain the waypoint following flag
                    targetData.followingWaypoints = true;
                    
                    // Update status message
                    snprintf((char*)navStatus.statusMessage, sizeof(((NavStatus*)0)->statusMessage), "Starting waypoint sequence again");
                    
                    //LOG_NAV("Last waypoint reached, looping back");
                    //LOG_NAV("Looping back to waypoint 0. targetLat=%.7f, targetLon=%.7f", 
                    //       waypoints[0].latitude, waypoints[0].longitude);
                }
                
                // Check if target distance has been reached
                float distanceTraveled = navStatus.distanceTraveled;
                float targetDistance = navStatus.targetDistance;
                
                if (targetDistance > 0 && distanceTraveled >= targetDistance) {
                    // Target distance reached, stop navigation
                    targetData.followingWaypoints = false;
                    navStatus.autonomousMode = false;
                    navStatus.destinationReached = true;
                    
                    // Update status message
                    snprintf((char*)navStatus.statusMessage, sizeof(((NavStatus*)0)->statusMessage), "Target distance reached");
                    
                    //LOG_DEBUG("Target distance of %.2f m reached (traveled: %.2f m)", 
                    //         targetDistance, distanceTraveled);
                }
            } else {
                // Single waypoint mode - destination reached
                navStatus.autonomousMode = false;
                navStatus.isPaused = false;
                navStatus.destinationReached = true;
                
                // Update status message
                snprintf((char*)navStatus.statusMessage, sizeof(((NavStatus*)0)->statusMessage), "Destination reached");

                LOG_DEBUG("Single destination reached, stopping navigation");
            }
            
            xSemaphoreGive(navDataMutex);
        }
        xSemaphoreGive(waypointMutex);
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
    //LOG_DEBUG("getNavStatus");
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
    //LOG_NAV("addWaypoint");

    NavCommand cmd;
    cmd.type = NAV_CMD_ADD_WAYPOINT;
    cmd.waypoint.latitude = latitude;
    cmd.waypoint.longitude = longitude;

    // Log queue state before sending
    UBaseType_t queueSpacesAvailable = uxQueueSpacesAvailable(navCommandQueue);
    LOG_NAV("navCommandQueue spaces, %d", queueSpacesAvailable);
    
    if (xQueueSend(navCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        LOG_ERROR("navCommandQueue fail in addWaypoint");
        return false;
    }
    
    //LOG_NAV("Waypoint command successfully queued");
    return true;
}

// Clear all waypoints (thread-safe)
bool clearWaypoints() {
    //LOG_DEBUG("clearWaypoints");
    NavCommand cmd;
    cmd.type = NAV_CMD_CLEAR_WAYPOINTS;
    
    if (xQueueSend(navCommandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        LOG_ERROR("navCommandQueue in clearWaypoints");
        return false;
    }
    
    return true;
}

// Get current waypoint count (thread-safe)
int getWaypointCount() {
    //LOG_DEBUG("getWaypointCount");
    int count = 0;
    
    if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        count = waypointCount;
        xSemaphoreGive(waypointMutex);
    }
    
    return count;
}

// Calculate bearing between two coordinates
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    //LOG_DEBUG("calculateBearing");
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
    //LOG_DEBUG("calculateDistance");
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

void setNavStatusMessage(const char* message) {
    //LOG_DEBUG("setNavStatusMessage");
    if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Use snprintf to safely copy string to the fixed buffer
        snprintf((char*)navStatus.statusMessage, sizeof(((NavStatus*)0)->statusMessage), "%s", message);
        xSemaphoreGive(navDataMutex);
    } else {
        LOG_ERROR("navDataMutex fail in setNavStatusMessage");
    }
}