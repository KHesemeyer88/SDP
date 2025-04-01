#ifndef AVOIDANCE_H
#define AVOIDANCE_H

#include <Arduino.h>
#include "config.h"

// External variables (defined elsewhere in your project)
extern float lastFrontDist, lastLeftDist, lastRightDist;
extern String lastAvoidanceMessage;
extern Servo escServo;

    
// Define throttle values.
// const int FULL_REVERSE_THROTTLE = ESC_MAX_REV;         // Use maximum reverse throttle.
const int SLOW_FORWARD_THROTTLE = ESC_MIN_FWD + 15;      // A slow forward throttle above minimum.

// Revised obstacle avoidance function that controls both throttle and steering.
inline int applyObstacleAvoidance(int steeringAngle) {
    unsigned long now = millis();
    // Static variables to track front obstacle avoidance state.
    static bool frontAvoidActive = false;
    static bool sideAvoidActive = false;
    static unsigned long avoidStartTime = 0;
    
    // --- FRONT OBSTACLE LOGIC ---
    if (lastFrontDist > 0 && lastFrontDist < FRONT_STOP_THRESHOLD) {
        if (!frontAvoidActive) {
            // Start avoidance 
            frontAvoidActive = true;
            avoidStartTime = now;
            escServo.write(SLOW_FORWARD_THROTTLE);  // Command slow fwd throttle.
            steeringAngle = STEERING_CENTER + TURN_ANGLE; // turn right
            steeringServo.write(steeringAngle);
            lastAvoidanceMessage = "Front obstacle";
            // Already in avoidance mode.
            if (now - avoidStartTime < 500) {  // 
                escServo.write(SLOW_FORWARD_THROTTLE);
                steeringAngle = STEERING_CENTER + TURN_ANGLE; // turn right
                steeringServo.write(steeringAngle);
            } else {
                frontAvoidActive = false;
                lastAvoidanceMessage = "Resume navigation";
            }
        }
        return steeringAngle;
    }
    
    // --- SIDE OBSTACLE LOGIC ---
    // Avoiding left obstacle
    if (lastLeftDist > 0 && lastLeftDist < SIDE_AVOID_THRESHOLD) {
        if (!sideAvoidActive) {
            sideAvoidActive = true;
            avoidStartTime = now;
            steeringServo.write(STEERING_CENTER + TURN_ANGLE);
            escServo.write(SLOW_FORWARD_THROTTLE);
            lastAvoidanceMessage = "Left obstacle: steering right";
            if (now - avoidStartTime < 500) {
                steeringServo.write(STEERING_CENTER + TURN_ANGLE);
                escServo.write(SLOW_FORWARD_THROTTLE);
            } else {
                sideAvoidActive = false;
                lastAvoidanceMessage = "Resume navigation";
            }
        }
        return steeringAngle;
    }
    
    // Avoiding right obstacle
    if (lastRightDist > 0 && lastRightDist < SIDE_AVOID_THRESHOLD) {
        if (!sideAvoidActive) {
            sideAvoidActive = true;
            avoidStartTime = now;
            steeringServo.write(STEERING_CENTER - TURN_ANGLE);
            escServo.write(SLOW_FORWARD_THROTTLE);
            lastAvoidanceMessage = "Right obstacle: steering left";
            if (now - avoidStartTime < 500) {
                steeringServo.write(STEERING_CENTER - TURN_ANGLE);
                escServo.write(SLOW_FORWARD_THROTTLE);
            } else {
                sideAvoidActive = false;
                lastAvoidanceMessage = "Resume navigation";
            }
        return steeringAngle;
        }
    }
    
    // --- Normal Operation ---
    lastAvoidanceMessage = "";
    return STEERING_CENTER;
}

#endif // AVOIDANCE_H
