#include "rtos_tasks.h"
#include "rtos_control.h"
#include "config.h"
#include <ESP32Servo.h>
#include "logging.h"

// Control task variables
unsigned long lastCommandTime = 0;
bool drivingState = false;
unsigned long lastNonNeutralCommand = 0;
const unsigned long DRIVING_GRACE_PERIOD = 1500; // 1.5 seconds grace period

// Servo objects
extern Servo steeringServo;
extern Servo escServo;

// Control task - handles servo commands
void ControlTask(void *pvParameters) {
    // Initialize
    LOG_DEBUG("Control Task Started");
    
    // Center steering and set neutral throttle
    if (xSemaphoreTake(servoMutex, portMAX_DELAY) == pdTRUE) {
        steeringServo.write(STEERING_CENTER);
        escServo.write(ESC_NEUTRAL);
        LOG_DEBUG("Steering and ESC set neutral");
        xSemaphoreGive(servoMutex);
    }
    
    // Local command variable
    ControlCommand cmd;
    
    // Task loop
    for (;;) {
        // Check for new commands in queue
        if (commandQueue != NULL && xQueueReceive(commandQueue, &cmd, 0) == pdPASS) {
            // Process command based on type
            switch (cmd.type) {
                case CMD_MANUAL_CONTROL:
                    // Update driving state
                    drivingState = cmd.manual.isDriving;
                    
                    if (drivingState) {
                        lastNonNeutralCommand = millis();
                    }
                    
                    // Take mutex before accessing servos
                    if (xSemaphoreTake(servoMutex, portMAX_DELAY) == pdTRUE) {
                        // Map joystick values to servo values
                        float normalizedY = cmd.manual.throttle;
                        float normalizedX = cmd.manual.steering;
                        
                        // Calculate ESC (throttle) value
                        int escValue;
                        const float JOYSTICK_DEADZONE = 0.03f;
                        
                        if (abs(normalizedY) < JOYSTICK_DEADZONE) {
                            escValue = ESC_NEUTRAL;
                        }
                        else if (normalizedY > 0) {
                            escValue = ESC_NEUTRAL + normalizedY * (ESC_MAX_FWD - ESC_NEUTRAL);
                        }
                        else {
                            escValue = ESC_NEUTRAL + normalizedY * (ESC_NEUTRAL - ESC_MAX_REV);
                        }
                        
                        // Apply ESC deadzone
                        if (escValue < ESC_MIN_FWD && escValue > ESC_MIN_REV) {
                            escValue = ESC_NEUTRAL;
                        }
                        
                        // Calculate steering value
                        int steeringValue = STEERING_CENTER + normalizedX * STEERING_MAX;
                        
                        // Constrain values to valid ranges
                        steeringValue = constrain(steeringValue, STEERING_CENTER - STEERING_MAX, STEERING_CENTER + STEERING_MAX);
                        escValue = constrain(escValue, ESC_MAX_REV, ESC_MAX_FWD);
                        
                        // Apply to servos
                        steeringServo.write(steeringValue);
                        escServo.write(escValue);
                        lastCommandTime = millis();
                        
                        // Release mutex
                        xSemaphoreGive(servoMutex);
                    }
                    break;
                    
                // Other command types will be added later
                default:
                    // Unknown command type
                    break;
            }
        }
        
        // Safety timeouts - if no commands for a while, stop the car
        unsigned long currentTime = millis();
        if (currentTime - lastCommandTime > COMMAND_TIMEOUT_MS) {
            if (xSemaphoreTake(servoMutex, portMAX_DELAY) == pdTRUE) {
                LOG_ERROR("Command timeout - stopping vehicle");
                escServo.write(ESC_NEUTRAL);
                steeringServo.write(STEERING_CENTER);
                xSemaphoreGive(servoMutex);
            } else {
                LOG_ERROR("Failed to acquire servo mutex for timeout handling");
            }
        }
        
        // Update driving state - exit driving state after grace period
        if (drivingState && (currentTime - lastNonNeutralCommand > DRIVING_GRACE_PERIOD)) {
            LOG_DEBUG("Exiting driving state due to inactivity");
            drivingState = false;
        }
        
        // Small delay
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz update rate
    }
}