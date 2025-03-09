#ifndef AVOIDANCE_H
#define AVOIDANCE_H

#include <Arduino.h>
#include "config.h"

// External variables (defined elsewhere in your project)
extern float lastFrontDist, lastLeftDist, lastRightDist;
extern String lastAvoidanceMessage;
extern Servo escServo;

// Revised obstacle avoidance function that controls both throttle and steering.
inline int applyObstacleAvoidance(int SteeringAngle) {
    unsigned long now = millis();
    // Static variables to track front obstacle avoidance state.
    static bool frontAvoidActive = false;
    static unsigned long frontAvoidStartTime = 0;
    
    // Define throttle values.
    const int FULL_REVERSE_THROTTLE = ESC_MAX_REV;         // Use maximum reverse throttle.
    const int SLOW_FORWARD_THROTTLE = ESC_MIN_FWD + 10;      // A slow forward throttle above minimum.

    // --- FRONT OBSTACLE LOGIC ---
    if (lastFrontDist > 0 && lastFrontDist < FRONT_STOP_THRESHOLD) {
        if (!frontAvoidActive) {
            // Start the avoidance sequence: trigger reverse.
            frontAvoidActive = true;
            frontAvoidStartTime = now;
            escServo.write(ESC_MAX_REV);  // Command full reverse throttle.
            lastAvoidanceMessage = "Front obstacle";
            return STEERING_CENTER;                // Keep steering centered during reverse.
        } else {
            // Already in avoidance mode.
            if (now - frontAvoidStartTime < 500) {  // 100 ms reverse period.
                escServo.write(ESC_MAX_REV);
                lastAvoidanceMessage = "Front obstacle";
                return STEERING_CENTER;
            } else {
                // After reverse period, resume forward motion at slow speed.
                escServo.write(SLOW_FORWARD_THROTTLE);
                // Adjust steering to turn away from the obstacle.
                if (lastLeftDist > lastRightDist) {
                    lastAvoidanceMessage = "Front obstacle: turning left";
                    frontAvoidActive = false; // Clear avoidance state.
                    return STEERING_CENTER - TURN_ANGLE;
                } else {
                    lastAvoidanceMessage = "Front obstacle: turning right";
                    frontAvoidActive = false;
                    return STEERING_CENTER + TURN_ANGLE;
                }
            }
        }
    } else {
        // No front obstacle detected: clear the avoidance state.
        frontAvoidActive = false;
    }
    
    // --- SIDE OBSTACLE LOGIC ---
    if (lastLeftDist > 0 && lastLeftDist < SIDE_AVOID_THRESHOLD) {
        steeringServo.write(STEERING_CENTER + TURN_ANGLE);
        escServo.write(SLOW_FORWARD_THROTTLE);
        lastAvoidanceMessage = "Left obstacle: steering right";
        return STEERING_CENTER + TURN_ANGLE;
    }
    if (lastRightDist > 0 && lastRightDist < SIDE_AVOID_THRESHOLD) {
        steeringServo.write(STEERING_CENTER - TURN_ANGLE);
        escServo.write(SLOW_FORWARD_THROTTLE);
        lastAvoidanceMessage = "Right obstacle: steering left";
        return STEERING_CENTER - TURN_ANGLE;
    }
    
    // --- Normal Operation ---
    lastAvoidanceMessage = "";
    return STEERING_CENTER;
}

#endif // AVOIDANCE_H

