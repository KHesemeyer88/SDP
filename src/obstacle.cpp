#include "obstacle.h"
#include "config.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Wire.h>
#include "LIDARLite_v4LED.h"
#include <string.h>

// LIDAR sensor object and instantiate a second I2C bus.
static LIDARLite_v4LED myLIDAR;
static TwoWire lidarWire = TwoWire(1);

// store FILTER_SAMPLES sensor readings.
static float frontReadings[FILTER_SAMPLES];
static int readIndex = 0;

// initialized to a safe value.
float lastFrontDist = 500;

// Global obstacle override structure.
ObstacleOverride obstacleOverride = { false, STEERING_CENTER, ESC_NEUTRAL };

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
// Returns a distance in centimeters, clamped to [0, 500].
static float readLIDAR() {
    float distance = myLIDAR.getDistance();
    if (distance < 0) distance = 500;
    if (distance > 500) distance = 500;
    return distance;
}

// ObstacleTask: 
// Periodically reads the front sensor, computes a median reading, and sets override values.
void ObstacleTask(void *pvParameters) {
    // First, initialize the sonar sensor.
    initializeSonar();
    uint8_t avoidanceDirection = 0; // 0=right, 1=left for alternating
    uint32_t lastObstacleTime = 0;
    const uint32_t CLEARANCE_DELAY_MS = 2000; // 2 seconds of clear space before resume
    
    for (;;) {
        // Wait for the update interval.
        vTaskDelay(pdMS_TO_TICKS(SONAR_UPDATE_INTERVAL));
        
        // Get a new reading from the sensor.
        float distance = readLIDAR();
        // Save the reading in the circular buffer.
        frontReadings[readIndex] = distance;
        readIndex = (readIndex + 1) % FILTER_SAMPLES;
        
        // Compute a median from the circular buffer.
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
            // New obstacle detected
            if (!obstacleOverride.active) {
                avoidanceDirection = !avoidanceDirection; // Alternate turn direction
                lastObstacleTime = millis();
            }

            // Apply avoidance maneuver
            obstacleOverride.active = true;
            obstacleOverride.overrideSteeringAngle = STEERING_CENTER +
                (avoidanceDirection ? -TURN_ANGLE : TURN_ANGLE); 
            obstacleOverride.overrideThrottle = ESC_MIN_FWD + 15; // slow speed
            }
        
        // Clear condition 
        else {
            obstacleOverride.active = false;
        }
    }
}
