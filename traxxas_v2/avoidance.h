// avoidance.h
#ifndef AVOIDANCE_H
#define AVOIDANCE_H

#include <Arduino.h>
#include "config.h" // For constants like FRONT_STOP_THRESHOLD

// Avoidance states
enum AvoidanceState {
    NO_OBSTACLE,
    REVERSING,
    TURNING,
    RESUMING,
    SIDE_AVOIDANCE
};

// Variables needed for avoidance logic
extern float lastFrontDist, lastLeftDist, lastRightDist;
extern String lastAvoidanceMessage;
extern unsigned long lastAvoidanceTime;
extern Servo steeringServo, escServo;
extern unsigned long lastUpdateTime;

// Current state information
static AvoidanceState avoidanceState = NO_OBSTACLE;
static unsigned long avoidanceStartTime = 0;
static bool avoidLeft = false; // True if avoiding left obstacle, false for right

// Functions
bool checkObstacles(int steeringAngle) {
    // Check for obstacles and activate the state machine if necessary
    if (lastFrontDist > 0 && lastFrontDist < FRONT_STOP_THRESHOLD) {
        // Front obstacle detected
        if (avoidanceState == NO_OBSTACLE) {
            avoidanceState = REVERSING;
            avoidanceStartTime = millis();
            lastAvoidanceMessage = "Front obstacle detected - reversing";
            return true;
        }
    } else if (lastLeftDist > 0 && lastLeftDist < SIDE_AVOID_THRESHOLD) {
        // Left obstacle detected
        if (avoidanceState == NO_OBSTACLE) {
            avoidanceState = SIDE_AVOIDANCE;
            avoidLeft = true;
            avoidanceStartTime = millis();
            lastAvoidanceMessage = "Avoiding left obstacle";
            return true;
        }
    } else if (lastRightDist > 0 && lastRightDist < SIDE_AVOID_THRESHOLD) {
        // Right obstacle detected
        if (avoidanceState == NO_OBSTACLE) {
            avoidanceState = SIDE_AVOIDANCE;
            avoidLeft = false;
            avoidanceStartTime = millis();
            lastAvoidanceMessage = "Avoiding right obstacle";
            return true;
        }
    } else {
        // No obstacles detected
        if (avoidanceState != NO_OBSTACLE) {
            // Reset the state machine if no obstacles are detected
            avoidanceState = NO_OBSTACLE;
            lastAvoidanceMessage = "";
        }
    }
    return false;
}

// Handle avoidance state machine
int handleAvoidance(int baseSteeringAngle) {
    int steeringAngle = baseSteeringAngle;
    int speed = 128; // Base speed
    
    // State machine logic
    switch (avoidanceState) {
        case NO_OBSTACLE: {
            // Normal navigation
            escServo.write(map(speed, 0, 255, ESC_MIN_FWD, ESC_MAX_FWD));
            break;
        }

        case REVERSING: {
            // Reverse for 500ms
            escServo.write(ESC_MAX_REV);
            if (millis() - avoidanceStartTime >= 500) {
                avoidanceState = TURNING;
                avoidanceStartTime = millis();
                escServo.write(ESC_NEUTRAL); // Stop before turning
            }
            break;
        }

        case TURNING: {
            // Turn right for 1 second
            steeringAngle = STEERING_CENTER + TURN_ANGLE; // Turn right
            escServo.write(map(128, 0, 255, ESC_MIN_FWD, ESC_MAX_FWD)); // Move forward
            if (millis() - avoidanceStartTime >= 1000) {
                avoidanceState = RESUMING;
                avoidanceStartTime = millis();
            }
            break;
        }

        case RESUMING: {
            // Resume normal navigation for 1 second
            steeringAngle = STEERING_CENTER; // Center steering
            escServo.write(map(128, 0, 255, ESC_MIN_FWD, ESC_MAX_FWD)); // Move forward
            if (millis() - avoidanceStartTime >= 1000) {
                avoidanceState = NO_OBSTACLE;
                lastAvoidanceMessage = "";
            }
            break;
        }

        case SIDE_AVOIDANCE: {
            // Steer away from the side obstacle
            if (avoidLeft) {
                steeringAngle = STEERING_CENTER - TURN_ANGLE; // Steer left
            } else {
                steeringAngle = STEERING_CENTER + TURN_ANGLE; // Steer right
            }
            escServo.write(map(128, 0, 255, ESC_MIN_FWD, ESC_MAX_FWD)); // Move forward
            if (millis() - avoidanceStartTime >= 1000) {
                avoidanceState = NO_OBSTACLE;
                lastAvoidanceMessage = "";
            }
            break;
        }
    }

    // Global safety timeout for any avoidance state
    if (avoidanceState != NO_OBSTACLE && 
        (millis() - avoidanceStartTime > MAX_AVOIDANCE_TIME)) {
        avoidanceState = NO_OBSTACLE;
        lastAvoidanceMessage = "Avoidance timeout - resuming";
        escServo.write(ESC_NEUTRAL); // Briefly stop to ensure safety
        delay(100); 
    }

    lastUpdateTime = millis();
    return steeringAngle;
}

// Get current avoidance state
AvoidanceState getAvoidanceState() {
    return avoidanceState;
}

// Reset avoidance state
void resetAvoidance() {
    avoidanceState = NO_OBSTACLE;
    lastAvoidanceMessage = "";
}

#endif // AVOIDANCE_H