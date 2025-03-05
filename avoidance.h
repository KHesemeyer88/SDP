#ifndef AVOIDANCE_H
#define AVOIDANCE_H

#include <Arduino.h>
#include "config.h"  

extern float lastFrontDist, lastLeftDist, lastRightDist;
extern String lastAvoidanceMessage;
extern Servo escServo;

enum AvoidanceState { NO_AVOID, FRONT_AVOID, LEFT_AVOID, RIGHT_AVOID };
static AvoidanceState avoidanceState = NO_AVOID;
static unsigned long avoidanceStartTime = 0;
const unsigned long AVOIDANCE_DURATION_MS = 1000; // Duration of avoidance action in milliseconds

inline int applyObstacleAvoidance(int baseSteeringAngle) {
    unsigned long now = millis();
    
    // If already in avoidance state, continue it until the duration elapses.
    if (avoidanceState != NO_AVOID) {
        if (now - avoidanceStartTime < AVOIDANCE_DURATION_MS) {
            switch (avoidanceState) {
                case FRONT_AVOID:
                    // Continue reverse command and steer away
                    escServo.write(ESC_MAX_REV);
                    if (lastLeftDist > lastRightDist) {
                        lastAvoidanceMessage = "Avoiding front obstacle: turning left";
                        return STEERING_CENTER - TURN_ANGLE;
                    } else {
                        lastAvoidanceMessage = "Avoiding front obstacle: turning right";
                        return STEERING_CENTER + TURN_ANGLE;
                    }
                
                case LEFT_AVOID:
                    lastAvoidanceMessage = "Avoiding left obstacle: steering right";
                    return STEERING_CENTER + TURN_ANGLE;
                
                case RIGHT_AVOID:
                    lastAvoidanceMessage = "Avoiding right obstacle: steering left";
                    return STEERING_CENTER - TURN_ANGLE;
                
                default:
                    break;
            }
        } else {
            // Timeout reached: exit avoidance mode
            avoidanceState = NO_AVOID;
            lastAvoidanceMessage = "";
        }
    }
    
    // Not in avoidance state; check sensors for new obstacles:
    if (lastFrontDist > 0 && lastFrontDist < FRONT_STOP_THRESHOLD) {
        avoidanceState = FRONT_AVOID;
        avoidanceStartTime = now;
        escServo.write(ESC_MAX_REV);
        if (lastLeftDist > lastRightDist) {
            lastAvoidanceMessage = "Front obstacle: turning left";
            return STEERING_CENTER - TURN_ANGLE;
        } else {
            lastAvoidanceMessage = "Front obstacle: turning right";
            return STEERING_CENTER + TURN_ANGLE;
        }
    }
    else if (lastLeftDist > 0 && lastLeftDist < SIDE_AVOID_THRESHOLD) {
        avoidanceState = LEFT_AVOID;
        avoidanceStartTime = now;
        lastAvoidanceMessage = "Left obstacle: steering right";
        return STEERING_CENTER + TURN_ANGLE;
    }
    else if (lastRightDist > 0 && lastRightDist < SIDE_AVOID_THRESHOLD) {
        avoidanceState = RIGHT_AVOID;
        avoidanceStartTime = now;
        lastAvoidanceMessage = "Right obstacle: steering left";
        return STEERING_CENTER - TURN_ANGLE;
    }
    
    return baseSteeringAngle;
}

inline void resetAvoidance() {
    avoidanceState = NO_AVOID;
    lastAvoidanceMessage = "";
}

#endif // AVOIDANCE_H
