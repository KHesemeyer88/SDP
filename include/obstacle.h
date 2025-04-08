#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "config.h"

// hold override values when an obstacle is detected.
typedef struct {
    bool active;                // True if an obstacle is detected.
    int overrideSteeringAngle;  // Override steering angle.
    int overrideThrottle;       // Override throttle value.
} ObstacleOverride;

extern ObstacleOverride obstacleOverride;

// hold the front sensor reading (in cm)
extern float lastFrontDist;

// Function prototype for the obstacle detection task
void ObstacleTask(void *pvParameters);

#endif // OBSTACLE_H