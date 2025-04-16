#include "obstacle.h"
#include "config.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Wire.h>
#include "LIDARLite_v4LED.h"
#include <string.h>
#include "navigation.h"

// LIDAR sensor object and instantiate a second I2C bus.
static LIDARLite_v4LED myLIDAR;
static TwoWire lidarWire = TwoWire(1);

// store FILTER_SAMPLES sensor readings.
static float frontReadings[FILTER_SAMPLES] = {0};
static int readIndex = 0;

// initialized to a safe value.
float lastFrontDist = 500;
static uint32_t exitDelay = 0;
const uint32_t exitDelay_end = 150; // in ms

// Global obstacle override structure.
ObstacleOverride obstacleOverride = { false, STEERING_CENTER, ESC_NEUTRAL };
SemaphoreHandle_t obstacleMutex = xSemaphoreCreateMutex();


// Initialize the sonar sensor (using the LIDAR sensor).
static void initializeSonar() {
    // Start the second I2C bus using the defined pins.
    lidarWire.begin(LIDAR_SDA_PIN, LIDAR_SCL_PIN);
    // Set I2C clock speed for faster communication.
    lidarWire.setClock(400000UL);
    
    // Initialize the LIDAR sensor.
    if (myLIDAR.begin(LIDARLITE_ADDR_DEFAULT, lidarWire)) {
        myLIDAR.configure(0);
        // Use 0 acquisitions for speed.
        uint8_t dataByte = 0x00;
        myLIDAR.write(0xEB, &dataByte, 1);
    }
    
    // Fill the circular buffer with the maximum safe distance.
    for (int i = 0; i < FILTER_SAMPLES; i++) {
        frontReadings[i] = 500;
    }
}

// Read a distance measurement from the LIDAR sensor.
// Returns a distance in centimeters, [2, 500].
static float readLIDAR() {
    float distance = myLIDAR.getDistance();
    if (distance < 2) distance = 500;
    if (distance > 500) distance = 500;
    return distance;
}

// ObstacleTask: 
// Periodically reads the front sensor, computes reading, and sets override values.
void ObstacleTask(void *pvParameters) {
    // First, initialize the sonar sensor.
    initializeSonar();
    uint8_t avoidanceDirection = 0; // 0=right, 1=left for alternating
    static uint32_t lastObstacleTime = 0;
    
    for (;;) {
        // Wait for the update interval.
        vTaskDelay(pdMS_TO_TICKS(SONAR_UPDATE_INTERVAL));
        
        
        // Get a new reading from the sensor.
        float distance = readLIDAR();
        // Save the reading in the buffer.
        frontReadings[readIndex] = distance;
        readIndex = (readIndex + 1) % FILTER_SAMPLES;
        
        // Compute a median from the buffer.
        float sortedReadings[FILTER_SAMPLES];
        memcpy(sortedReadings, frontReadings, sizeof(frontReadings));
        for (int i = 0; i < FILTER_SAMPLES - 1; i++) {
            for (int j = i + 1; j < FILTER_SAMPLES; j++) {
                if (sortedReadings[j] < sortedReadings[i]) {
                    float temp = sortedReadings[i];
                    sortedReadings[i] = sortedReadings[j];
                    sortedReadings[j] = temp;
                }
            }
        }
        lastFrontDist = sortedReadings[FILTER_SAMPLES / 2];
  

        // Obstacle state machine
        if (lastFrontDist < FRONT_STOP_THRESHOLD) {
            // obstacle detected
            
            if (!obstacleOverride.active) {
                avoidanceDirection = !avoidanceDirection; // Alternate turn direction
                lastObstacleTime = millis();
                /*
                if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    navStatus.isPaused = true;
                    xSemaphoreGive(navDataMutex);
                }
                */
            }
            exitDelay = 0;
            // Apply avoidance maneuver
            
            if (xSemaphoreTake(obstacleMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                obstacleOverride.active = true;
                obstacleOverride.overrideSteeringAngle = STEERING_CENTER + (avoidanceDirection ? -TURN_ANGLE : TURN_ANGLE);
                obstacleOverride.overrideThrottle = ESC_MIN_FWD + 10; // slow speed
                xSemaphoreGive(obstacleMutex);
            }
        }
        
        // Clear condition 
        else {
            if (obstacleOverride.active && exitDelay == 0) {
                exitDelay = millis();
            }
            
            if (exitDelay > 0 && (millis() - exitDelay > exitDelay_end)) {
                obstacleOverride.active = false;
                exitDelay = 0;

                if (xSemaphoreTake(navDataMutex, pdMS_TO_TICKS(5))) {
                    navStatus.isPaused = false;
                    xSemaphoreGive(navDataMutex);
                }
            }
        }
    }
}

