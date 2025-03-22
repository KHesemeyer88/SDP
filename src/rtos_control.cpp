#include "rtos_tasks.h"
#include "rtos_control.h"
#include "config.h"
#include <ESP32Servo.h>
#include "logging.h"
#include "navigation.h"
#include "gnss.h"

// Control task variables
unsigned long lastCommandTime = 0;
bool drivingState = false;
unsigned long lastNonNeutralCommand = 0;
const unsigned long DRIVING_GRACE_PERIOD = 1500; // 1.5 seconds grace period

// For reverse handling
bool inReverseMode = false;
bool reverseSequenceActive = false;
unsigned long reverseSequenceStartTime = 0;
int reverseSequenceStep = 0;

// For PI control
static float cumulativeError = 0;
static int lastEscCommand = ESC_MIN_FWD;

// Servo objects
extern Servo steeringServo;
extern Servo escServo;

// Control task function
void ControlTask(void *pvParameters) {
    ControlCommand currentCmd;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CONTROL_UPDATE_FREQUENCY);
    
    // Local variables for position, heading, target
    float currentLat, currentLon, currentSpeed, currentHeading;
    float targetLat, targetLon;
    bool autonomousActive, isPaused, followingWaypoints;
    
    LOG_DEBUG("Control task started");
    
    // Initialize time for consistent frequency
    xLastWakeTime = xTaskGetTickCount();
    
    // Task loop
    while (true) {
        unsigned long currentTime = millis();
        
        // Check for manual control commands
        if (xQueueReceive(commandQueue, &currentCmd, 0) == pdTRUE) {
            //LOG_DEBUG("Received command queue");
            // Process the command based on type
            if (currentCmd.type == CMD_MANUAL_CONTROL) {
                // Manual control command received
                //LOG_DEBUG("Manual control command received");
                // Update driving state and timestamp
                drivingState = currentCmd.manual.isDriving;
                if (drivingState) {
                    lastNonNeutralCommand = currentTime;
                }
                
                // Apply manual control if we're not in autonomous mode
                if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    // Check if we're in autonomous mode
                    //LOG_DEBUG("Checking if we're in autonomous mode");
                    autonomousActive = navStatus.autonomousMode;
                    xSemaphoreGive(navDataMutex);
                    
                    if (!autonomousActive) {
                        //LOG_DEBUG("Entering manual control in controlTask");
                        // Apply manual control commands to servos
                        if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                            //LOG_DEBUG("Received servoMutex");
                            // Map joystick to servo values
                            int steeringValue = STEERING_CENTER + (currentCmd.manual.steering * STEERING_MAX);
                            
                            // Apply steering limit
                            steeringValue = constrain(steeringValue, 
                                              STEERING_CENTER - STEERING_MAX, 
                                              STEERING_CENTER + STEERING_MAX);
                            
                           // Apply throttle based on joystick position
                            int throttleValue;
                            if (currentCmd.manual.throttle > 0) {
                                //LOG_DEBUG("Forward throttle value set");
                                // Forward
                                inReverseMode = false; // Exit reverse mode if forward requested
                                throttleValue = ESC_NEUTRAL + (currentCmd.manual.throttle * (ESC_MAX_FWD - ESC_NEUTRAL));
                            } else if (currentCmd.manual.throttle < 0) {
                                //LOG_DEBUG("Neg. throttle detected");
                                // Reverse - check if we need to start sequence
                                if (!inReverseMode && !reverseSequenceActive) {
                                    //LOG_DEBUG("Setting rev. sequence params");
                                    reverseSequenceActive = true;
                                    reverseSequenceStep = 0;
                                    reverseSequenceStartTime = currentTime;
                                    throttleValue = ESC_NEUTRAL; // Start with neutral
                                } else if (inReverseMode) {
                                    // Already in reverse mode, can apply reverse directly
                                    throttleValue = ESC_NEUTRAL + (currentCmd.manual.throttle * (ESC_NEUTRAL - ESC_MAX_REV));
                                    //LOG_DEBUG("Reverse mode active: throttle=%.2f, output=%d", currentCmd.manual.throttle, throttleValue);
                                } else {
                                    //LOG_DEBUG("Let sequence handler control throttle");
                                    // In middle of sequence, let sequence handler control throttle
                                    throttleValue = ESC_NEUTRAL;
                                }
                            } else {
                                //LOG_DEBUG("Neutral");
                                // Neutral
                                throttleValue = ESC_NEUTRAL;
                            }

                            // Apply servo commands (ESC only if not in reverse sequence)
                            steeringServo.write(steeringValue);
                            if (!reverseSequenceActive) {
                                //LOG_DEBUG("Throttle applied: %d", throttleValue);
                                escServo.write(throttleValue);
                            }

                            // Process reverse sequence if active
                            if (reverseSequenceActive) {
                                //LOG_DEBUG("Entering reverse sequence, case: %d", reverseSequenceStep);
                                unsigned long elapsedTime = currentTime - reverseSequenceStartTime;
                                
                                switch (reverseSequenceStep) {
                                    case 0: // First neutral (these intervals are based on test code experiments)
                                    // it's possible shorter intervals may work as well
                                        escServo.write(ESC_NEUTRAL);
                                        if (elapsedTime >= 50) {
                                            //LOG_DEBUG("Reverse seq: First neutral complete");
                                            reverseSequenceStep = 1;
                                            reverseSequenceStartTime = currentTime;
                                        }
                                        break;
                                        
                                    case 1: // First reverse tap
                                        escServo.write(0); //experimenting showed that 0 works most robustly
                                        if (elapsedTime >= 50) {
                                            //LOG_DEBUG("Reverse seq: First tap complete");
                                            reverseSequenceStep = 2;
                                            reverseSequenceStartTime = currentTime;
                                        }
                                        break;
                                        
                                    case 2: // Second neutral
                                        escServo.write(ESC_NEUTRAL);
                                        if (elapsedTime >= 50) {
                                            //LOG_DEBUG("Reverse seq: Second neutral complete");
                                            reverseSequenceStep = 3;
                                            reverseSequenceStartTime = currentTime;
                                        }
                                        break;
                                        
                                    case 3: // Complete sequence
                                        //LOG_DEBUG("Reverse sequence complete - now in reverse mode");
                                        inReverseMode = true;
                                        reverseSequenceActive = false;
                                        // Apply the reverse throttle immediately
                                        // Start with moderate reverse
                                        escServo.write(70);
                                        vTaskDelay(5);
                                        break;
                                }
                            }
                            
                            // Update command timestamp
                            lastCommandTime = currentTime;
                            
                            xSemaphoreGive(servoMutex);
                        }
                    }
                }
            }
            // Add other command types as needed
        }
        
        // Check navigation status for autonomous control
        NavStatus nav;
        if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            nav.autonomousMode = navStatus.autonomousMode;
            nav.isPaused = navStatus.isPaused;
            nav.targetPace = navStatus.targetPace;
            xSemaphoreGive(navDataMutex);

            // Add state transition tracking
            static bool lastAutoMode = false;
            static bool lastPaused = false;
            if (nav.autonomousMode != lastAutoMode || nav.isPaused != lastPaused) {
                LOG_NAV("Control task detected navigation state change: auto=%d, paused=%d", 
                        nav.autonomousMode, nav.isPaused);
                lastAutoMode = nav.autonomousMode;
                lastPaused = nav.isPaused;
            }
        }

        // Add periodic logging of control values in autonomous mode
        static unsigned long lastControlLog = 0;
        if (nav.autonomousMode && !nav.isPaused && millis() - lastControlLog > 500) {
            lastControlLog = millis();
            LOG_NAV("Control values: speed=%.2f, targetSpeed=%.2f, heading=%.2f°", 
                    currentSpeed, nav.targetPace, currentHeading);
        }
        
        // If we're in autonomous mode and not paused, apply navigation control
        if (nav.autonomousMode && !nav.isPaused) {
            // Get current GNSS data
            if (xSemaphoreTake(gnssMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                currentLat = gnssData.latitude;
                currentLon = gnssData.longitude;
                currentSpeed = gnssData.speed;
                currentHeading = gnssData.heading;
                xSemaphoreGive(gnssMutex);
                // Make a direct call to get fresh heading: THIS WAS MAYBE CAUSING PROBLEMS
                //currentHeading = myGPS.getHeading() / 100000.0; // Convert to degrees
            }
            
            // Get target data
            if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                targetLat = targetData.targetLat;
                targetLon = targetData.targetLon;
                followingWaypoints = targetData.followingWaypoints;
                xSemaphoreGive(waypointMutex);
            }
            
            // Calculate steering angle based on current and target positions
            int steeringAngle = calculateSteeringAngle(currentLat, currentLon, targetLat, targetLon, currentHeading);
            
            // Calculate throttle value based on pace control
            int throttleValue = calculateThrottle(currentSpeed, nav.targetPace);
            
            // Apply control commands
            if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                steeringServo.write(steeringAngle);
                escServo.write(throttleValue);
                xSemaphoreGive(servoMutex);
            }
            
            // Update command timestamp
            lastCommandTime = currentTime;
        }
        else if (nav.autonomousMode && nav.isPaused) {
            // If paused, set motor to neutral but maintain steering
            if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                escServo.write(ESC_NEUTRAL);
                xSemaphoreGive(servoMutex);
            }
        }
        
        // Safety timeout - stop if no recent commands
        if (currentTime - lastCommandTime > COMMAND_TIMEOUT_MS) {
            if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                escServo.write(ESC_NEUTRAL);
                steeringServo.write(STEERING_CENTER);
                xSemaphoreGive(servoMutex);
            }
        }
        
        // Use vTaskDelayUntil to ensure consistent timing
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Calculate steering angle based on current position, target, and heading
int calculateSteeringAngle(float currentLat, float currentLon, float targetLat, float targetLon, float currentHeading) {
    // Calculate bearing to target
    float bearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
    
    // Calculate heading error (difference between bearing and current heading)
    float headingError = bearing - currentHeading;
    
    // Normalize heading error to -180 to 180 range
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;
    
    // Determine correction factor based on GNSS quality
    float correctionFactor = 0.35; // Default value
    
    // For large errors (> 45°), use full steering capability
    if (abs(headingError) > 45.0) {
        // Apply maximum steering in the appropriate direction
        return (headingError > 0) ? 
                STEERING_CENTER + STEERING_MAX : 
                STEERING_CENTER - STEERING_MAX;
    } 
    // For moderate to small errors, use proportional control
    else {
        // Linear scaling for smooth transition to maximum steering
        float scaleFactor = correctionFactor * (45.0 / STEERING_MAX);
        int steeringAngle = STEERING_CENTER + (headingError * scaleFactor);
        
        // Apply steering limits
        return constrain(steeringAngle, 
                         STEERING_CENTER - STEERING_MAX, 
                         STEERING_CENTER + STEERING_MAX);
    }
}

// Calculate throttle value based on current speed and target pace
int calculateThrottle(float currentSpeed, float targetPace) {
    // Only adjust speed if we have a target pace
    if (targetPace <= 0) {
        return ESC_NEUTRAL; // Stop if no target pace
    }
    
    // Static variables for PI control
    static float cumulativeError = 0;
    static int lastEscCommand = ESC_MIN_FWD;
    
    // Calculate pace error
    float paceError = targetPace - currentSpeed;
    int adjustmentValue = 0;
    
    // Vehicle is stopped or moving very slowly (GPS noise)
    if (currentSpeed < 0.5) {
        // Initial push to overcome inertia
        lastEscCommand = ESC_MIN_FWD + 15;
        cumulativeError = 0; // Reset integral term
    }
    // Normal PI control operation when vehicle is moving
    else {
        // Proportional component
        float pComponent = paceError * 1.5;
        
        // Integral component with slower accumulation
        cumulativeError = constrain(cumulativeError + paceError * 0.03, -3.0, 5.0);
        
        // Integral gain
        float iComponent = cumulativeError * 1.0;
        
        // Combined adjustment
        adjustmentValue = (int)(pComponent + iComponent);
        
        // Constrain adjustment to prevent lurching
        adjustmentValue = constrain(adjustmentValue, -1, 1);
        
        // Calculate new ESC value
        lastEscCommand = lastEscCommand + adjustmentValue;
    }
    
    // Minimum throttle floor based on target pace
    float minPowerLevel = ESC_MIN_FWD + (targetPace * 10.0);
    
    if (currentSpeed > 0.5 && lastEscCommand < minPowerLevel) {
        lastEscCommand = (int)minPowerLevel;
    }
    
    // Constrain final value
    return constrain(lastEscCommand, ESC_MIN_FWD, ESC_MAX_FWD);
}