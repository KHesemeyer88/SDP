#include "rtos_tasks.h"
#include "rtos_control.h"
#include "config.h"
#include <ESP32Servo.h>
#include "logging.h"
#include "navigation.h"
#include "gnss.h"
#include <PID_v1.h>
#include "obstacle.h"

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

// For resume navigation
static bool preObstacleActive = false;
static bool pathRecovery = false;
static float originalBearing = 0;

// Servo objects
extern Servo steeringServo;
extern Servo escServo;

// PID control variables
double pidInput = 0;     // GNSS speed in m/s
double pidSetpoint = 0;  // targetPace in m/s
double pidOutput = 0;    // throttle signal to ESC

// For tracking navigation state changes for PID reset
static bool prevAutonomousMode = false;
static bool prevPausedState = false;
static int mutexWait = 50;

// Initial tuning values (adjust as needed)
double Kp = 3.0, Ki = 1.0, Kd = 0.1;

// PID controller instance (DIRECT means output increases when error is positive)
PID speedPID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

// Control task function
void ControlTask(void *pvParameters) {
    //LOG_DEBUG("ControlTask");
    ControlCommand currentCmd;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CONTROL_UPDATE_FREQUENCY);
    
    // Local variables for position, heading, target
    float currentLat, currentLon, currentSpeed, currentHeading;
    float targetLat, targetLon;
    bool autonomousActive, isPaused, followingWaypoints;
    speedPID.SetMode(AUTOMATIC);

    speedPID.SetOutputLimits(ESC_MIN_FWD, ESC_MAX_FWD);  // Safe range
    speedPID.SetSampleTime(1000 / CONTROL_UPDATE_FREQUENCY);  // Match task rate

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
                if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(mutexWait)) == pdTRUE) {
                    // Check if we're in autonomous mode
                    //LOG_DEBUG("Checking if we're in autonomous mode");
                    autonomousActive = navStatus.autonomousMode;
                    xSemaphoreGive(navDataMutex);
                    
                    if (!autonomousActive) {
                        //LOG_DEBUG("Entering manual control in controlTask");
                        // Apply manual control commands to servos
                        if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(mutexWait)) == pdTRUE) {
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
            // Add other command types?
        }
        
        // Check navigation status for autonomous control
        NavStatus nav;
        if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(mutexWait)) == pdTRUE) {
            nav.autonomousMode = navStatus.autonomousMode;
            nav.isPaused = navStatus.isPaused;
            nav.targetPace = navStatus.targetPace;
            xSemaphoreGive(navDataMutex);

            // Add state transition tracking
            static bool lastAutoMode = false;
            static bool lastPausedState = false;
            if (nav.autonomousMode != lastAutoMode || nav.isPaused != lastPausedState) {
                // LOG_NAV("ControlTask state chg, %d, %d", 
                //         nav.autonomousMode, nav.isPaused);
                
                // If transitioning from active navigation to stopped/paused
                if ((!nav.autonomousMode && lastAutoMode) || (nav.isPaused && !lastPausedState)) {
                    // When navigation stops or pauses, put PID in manual mode
                    speedPID.SetMode(MANUAL);
                    pidOutput = ESC_MIN_FWD;    // Reset output to minimum
                    pidInput = 0;              // Reset input 
                    //LOG_NAV("PID controller disabled due to navigation stop/pause");
                } 
                // If transitioning from stopped/paused to active navigation
                else if ((nav.autonomousMode && !lastAutoMode) || (!nav.isPaused && lastPausedState)) {
                    // When navigation starts or resumes, put PID back in automatic mode
                    pidOutput = ESC_MIN_FWD;  // Start with minimum throttle
                    speedPID.SetMode(AUTOMATIC);
                    //LOG_NAV("PID controller enabled due to navigation start/resume");
                }
                
                lastAutoMode = nav.autonomousMode;
                lastPausedState = nav.isPaused;
            }
        }
        
        // If we're in autonomous mode and not paused, apply navigation control
        if (nav.autonomousMode && !nav.isPaused) {
            // Get current GNSS data
            if (xSemaphoreTake(gnssMutex, pdMS_TO_TICKS(mutexWait)) == pdTRUE) {
                unsigned long now = millis();
                unsigned long age = now - gnssData.gnssFixTime;
                
                currentLat = gnssData.latitude;
                currentLon = gnssData.longitude;
                currentSpeed = gnssData.speed;
                currentHeading = gnssData.heading;
                
                xSemaphoreGive(gnssMutex);
                LOG_DEBUG("position-control latency time, %lu", age); //THIS IS POSITION - CONTROL LATENCY!!!!
                // Add periodic logging of control values in autonomous mode
                static unsigned long lastLocLog = 0;
                if (millis() - lastLocLog > 500) {
                    lastLocLog = millis();
                    LOG_NAV("ControlTask data, %.7f, %.7f, %.2f, %.1f", 
                        currentLat, currentLon, currentSpeed, currentHeading);
                }
            }

            
            // Get target data
            if (xSemaphoreTake(waypointMutex, pdMS_TO_TICKS(mutexWait)) == pdTRUE) {
                targetLat = targetData.targetLat;
                targetLon = targetData.targetLon;
                followingWaypoints = targetData.followingWaypoints;
                xSemaphoreGive(waypointMutex);
            }

            // Obstacle avoidance
            bool isObstacleActive = false;
            if (xSemaphoreTake(obstacleMutex, pdMS_TO_TICKS(mutexWait))) {
                isObstacleActive = obstacleOverride.active;
                xSemaphoreGive(obstacleMutex);
            }

            if (isObstacleActive) {
                if (!preObstacleActive) {
                    originalBearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
                }
                preObstacleActive = true;

                // Direct servo control during avoidance
                if (xSemaphoreTake(obstacleMutex, pdMS_TO_TICKS(mutexWait))) {
                    steeringServo.write(obstacleOverride.overrideSteeringAngle);
                    escServo.write(obstacleOverride.overrideThrottle);
                    xSemaphoreGive(obstacleMutex);
                }
            
            } else { // normal navigation 
                if (pathRecovery) { // once avoidance completed, compare bearing 
                    float bearingError = originalBearing - currentHeading;
                    bearingError = (bearingError > 180) ? bearingError - 360 : 
                                  (bearingError < -180) ? bearingError + 360 : bearingError;
                    // If bearingError greater than 5 degrees, adjustment needed to go back on track
                    if (fabs(bearingError) > 5.0) {
                        int steeringAngle = STEERING_CENTER + (bearingError > 0 ? STEERING_MAX/2 : -STEERING_MAX/2);
                        steeringServo.write(steeringAngle);
                    } else {
                        pathRecovery = false;
                    }
                } 
            
                preObstacleActive = false;
                
                // Calculate steering angle based on current and target positions
                int steeringAngle = calculateSteeringAngle(currentLat, currentLon, targetLat, targetLon, currentHeading, nav.targetPace);
                
                pidInput = currentSpeed;
                pidSetpoint = nav.targetPace;
                
                static int lastThrottleValue = ESC_NEUTRAL;
                if (speedPID.Compute()) {
                    lastThrottleValue = (int)pidOutput;
                }

                steeringAngle = constrain(steeringAngle, STEERING_CENTER - STEERING_MAX, STEERING_CENTER + STEERING_MAX);
                static unsigned long lastLogTime = 0;
                unsigned long now = millis();
                if (now - lastLogTime > 200) {
                    LOG_NAV("steeringAngle, %d", steeringAngle);
                    lastLogTime = now;
                }
                lastThrottleValue = constrain(lastThrottleValue, ESC_MIN_FWD, ESC_MAX_FWD);
                
                // Apply control commands
                if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(mutexWait)) == pdTRUE) {
                    steeringServo.write(steeringAngle);
                    escServo.write(lastThrottleValue);
                    xSemaphoreGive(servoMutex);
                }
            }
            
            // Update command timestamp
            lastCommandTime = currentTime;
        }
        else if (nav.autonomousMode && nav.isPaused) {
            // If paused, set motor to neutral but maintain steering
            if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(mutexWait)) == pdTRUE) {
                escServo.write(ESC_NEUTRAL);
                steeringServo.write(STEERING_CENTER + TRIM_ANGLE);
                xSemaphoreGive(servoMutex);
            }
        }
        
        // Safety timeout - stop if no recent commands
        if (currentTime - lastCommandTime > COMMAND_TIMEOUT_MS) {
            if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(mutexWait)) == pdTRUE) {
                escServo.write(ESC_NEUTRAL);
                steeringServo.write(STEERING_CENTER + TRIM_ANGLE);
                xSemaphoreGive(servoMutex);
            }
        }
        LOG_DEBUG("ControlTask time, %lu", millis() - currentTime);
        // Use vTaskDelayUntil to ensure consistent timing
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Calculate steering angle based on current position, target, heading, target pace
int calculateSteeringAngle(float currentLat, float currentLon, float targetLat, float targetLon, float currentHeading, float targetPace) {
    float bearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
    float headingError = bearing - currentHeading;

    // Normalize heading error to -180 to 180 range
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;

    static unsigned long lastLogTime = 0;
    unsigned long now = millis();
    if (now - lastLogTime > 200) {
        LOG_NAV("headingError, %.0f", headingError);
        lastLogTime = now;
    }

    float correctionFactor = 0.6;

    // Determine max steering angle based on target pace
    int effectiveSteeringMax = (targetPace > 4.0) ? STEERING_MAX * 0.7 : STEERING_MAX;

    // Final angle with trim
    int steeringAngle = STEERING_CENTER + TRIM_ANGLE + round(correctionFactor * headingError);

    // Clamp to effective limits
    return constrain(steeringAngle,
                     STEERING_CENTER - effectiveSteeringMax,
                     STEERING_CENTER + effectiveSteeringMax);
}