#ifndef SONAR_H
#define SONAR_H

#include <Arduino.h>
#include "config.h" 
#include <Wire.h>
#include "LIDARLite_v4LED.h"

// Pins for LIDAR I2C bus
#define LIDAR_SDA_PIN 16  // Reusing the old sonar pins
#define LIDAR_SCL_PIN 17
#define LIDARLITE_ADDR_DEFAULT 0x62  // Default I2C address for LIDAR-Lite v4

// Sonar filter arrays (still used for LIDAR readings)
extern float frontReadings[];
extern float leftReadings[];
extern float rightReadings[];
extern int readIndex;

// Distance measurements
extern float lastFrontDist, lastLeftDist, lastRightDist;
extern unsigned long lastSonarUpdate;
extern int currentSonar;

// LIDAR sensor object 
LIDARLite_v4LED myLIDAR;

// Use a second I2C bus for LIDAR (lidarWire)
TwoWire lidarWire = TwoWire(1); // Use I2C bus #1

// Initialize sensors - now exclusively LIDAR
void initializeSonar() {
    // Initialize LIDAR for front distance measurements
    
    // Start a second I2C bus for LIDAR
    lidarWire.begin(LIDAR_SDA_PIN, LIDAR_SCL_PIN);
    
    // Set I2C frequency to 400kHz for faster communication
    lidarWire.setClock(400000UL);
    
    // Tell LIDAR to use lidarWire for communications
    if (myLIDAR.begin(LIDARLITE_ADDR_DEFAULT, lidarWire)) {
        // Configure LIDAR for maximum speed
        myLIDAR.configure(0);
        
        // Optimize acquisition count for faster readings (5 is a good balance)
        // 0 = fastest but potentially less reliable
        // 5 = good balance of speed and reliability
        // 20 = default, most reliable but slow
        uint8_t dataByte = 0x00;  // Using 0 acquisitions for speed
        myLIDAR.write(0xEB, &dataByte, 1);
    }
    
    // Initialize reading arrays to maximum distance
    for (int i = 0; i < FILTER_SAMPLES; i++) {
        frontReadings[i] = 500;  // Max distance 5m for LIDAR
        leftReadings[i] = 500;   // Max values for side sensors (not used)
        rightReadings[i] = 500;  // Max values for side sensors (not used)
    }
}

// Read distance from LIDAR sensor
float readLIDAR(float* readings) {
    // Get distance reading from LIDAR in cm
    float distance = myLIDAR.getDistance();
    
    // Constrain to valid range (0-500cm)
    if (distance < 0) distance = 500;  // Invalid reading
    if (distance > 500) distance = 500; // Cap at max distance (5m)
    
    // Add to readings array
    readings[readIndex] = distance;
    
    // Calculate median to filter out noise
    float sortedReadings[FILTER_SAMPLES]; 
    memcpy(sortedReadings, readings, sizeof(float) * FILTER_SAMPLES);
    
    // Sort readings for median calculation
    for (int i = 0; i < FILTER_SAMPLES - 1; i++) {
        for (int j = i + 1; j < FILTER_SAMPLES; j++) {
            if (sortedReadings[j] < sortedReadings[i]) {
                float temp = sortedReadings[i];
                sortedReadings[i] = sortedReadings[j];
                sortedReadings[j] = temp;
            }
        }
    }
    
    // Return median reading
    return sortedReadings[FILTER_SAMPLES / 2];
}

// Update sensor readings (only LIDAR for front, sides set to max)
void updateSonarReadings() {
    if (millis() - lastSonarUpdate >= SONAR_UPDATE_INTERVAL) {
        // Read front LIDAR
        lastFrontDist = readLIDAR(frontReadings);
        
        // Set side distances to maximum (no obstacles)
        lastLeftDist = 500;
        lastRightDist = 500;
        
        readIndex = (readIndex + 1) % FILTER_SAMPLES;
        lastSonarUpdate = millis();
    }
}

#endif // SONAR_H