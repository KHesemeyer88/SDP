// navigation.h
#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "config.h"
#include "gnss.h"
#include "logging.h"
#include "rtos_tasks.h"

// Navigation task configuration
#define NAV_TASK_PRIORITY       9  // Below GNSS but above WebSocket?
#define NAV_TASK_STACK_SIZE     4096
#define NAV_UPDATE_FREQUENCY    10 // 10Hz for navigation updates

// Control task configuration (for motion control)
#define CONTROL_UPDATE_FREQUENCY 20 // 20Hz for smoother control

// Task handles
extern TaskHandle_t navTaskHandle;

// Mutexes for protecting shared data
extern SemaphoreHandle_t navDataMutex;
extern SemaphoreHandle_t waypointMutex;

// Waypoint structure
typedef struct {
    float latitude;
    float longitude;
} Waypoint;

// Navigation status structure
typedef struct {
    bool autonomousMode;      // Whether navigation is active
    bool isPaused;            // Whether navigation is paused
    float distanceTraveled;   // Total distance traveled in meters
    float currentPace;        // Current speed in m/s
    float averagePace;        // Average speed in m/s
    unsigned long elapsedTime; // Elapsed time in milliseconds
    float distanceToWaypoint; // Distance to current waypoint in meters
    float targetPace;         // Target pace in m/s 
    float targetDistance;     // Target distance in meters
    int currentWaypoint;      // Current waypoint index
    int totalWaypoints;       // Total number of waypoints
    char statusMessage[64];   // Status/alert message with fixed buffer size
    bool destinationReached;  // Whether destination was reached
} NavStatus;

// Target data structure (shared between nav and control)
typedef struct {
    float targetLat;         // Target latitude
    float targetLon;         // Target longitude
    bool followingWaypoints; // Whether following multiple waypoints
} TargetData;

// Commands that can be sent to navigation task
typedef enum {
    NAV_CMD_START,           // Start navigation
    NAV_CMD_STOP,            // Stop navigation
    NAV_CMD_PAUSE,           // Pause navigation
    NAV_CMD_RESUME,          // Resume navigation
    NAV_CMD_RESET_STATS,     // Reset tracking stats
    NAV_CMD_ADD_WAYPOINT,    // Add a waypoint
    NAV_CMD_CLEAR_WAYPOINTS, // Clear all waypoints
    NAV_CMD_LOAD_ROUTE,      // Load route from SD
    NAV_CMD_SAVE_ROUTE       // Save route to SD
} NavCommandType;

// Navigation command structure
typedef struct {
    NavCommandType type;
    union {
        struct {
            float targetPace;
            float targetDistance;
            float latitude;   // For single waypoint navigation
            float longitude;  // For single waypoint navigation
        } start;
        struct {
            float latitude;
            float longitude;
        } waypoint;
        struct {
            char routeName[16]; // For loading/saving routes
        } route;
    };
} NavCommand;

// Global waypoints array
extern Waypoint waypoints[MAX_WAYPOINTS];
extern int waypointCount;

// Queue for navigation commands
extern QueueHandle_t navCommandQueue;

// Shared navigation status and target data
extern volatile NavStatus navStatus;
extern volatile TargetData targetData;

// Public function declarations
void NavigationTask(void *pvParameters);
bool initNavigation();

// Command interface functions (thread-safe)
bool startNavigation(float targetPace, float targetDistance, float lat, float lon);
bool startWaypointNavigation(float targetPace, float targetDistance);
bool stopNavigation();
bool pauseNavigation();
bool resumeNavigation();
bool resetNavigationStats();
bool addWaypoint(float latitude, float longitude);
bool clearWaypoints();
bool loadRouteFromSD(const char* routeName);
bool saveRouteToSD(const char* routeName);
int getWaypointCount();
NavStatus getNavStatus();

// Helper calculations
float calculateBearing(float lat1, float lon1, float lat2, float lon2);
float calculateDistance(float lat1, float lon1, float lat2, float lon2);

// Steering and throttle calculation functions
int calculateSteeringAngle(float currentLat, float currentLon, float targetLat, float targetLon, float currentHeading);
int calculateThrottle(float currentSpeed, float targetPace);

// Optional helper functions that might be useful to expose
//void handleWaypointReached();
void setNavStatusMessage(const char* message);

#endif // NAVIGATION_H