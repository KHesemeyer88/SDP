#ifndef SONAR_H
#define SONAR_H

#include <Arduino.h>
#include "config.h" 
#include "LIDARLite_v4LED.h"
#include <Wire.h>

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
    Serial.println("Initializing LIDAR sensor on dedicated I2C bus...");
    
    // Start a second I2C bus for LIDAR
    lidarWire.begin(LIDAR_SDA_PIN, LIDAR_SCL_PIN);
    
    // Tell LIDAR to use lidarWire for communications
    if (myLIDAR.begin(LIDARLITE_ADDR_DEFAULT, lidarWire)) {
        Serial.println("LIDAR sensor initialized successfully!");
    } else {
        Serial.println("LIDAR initialization failed! Check wiring.");
    }
    
    /* COMMENTED OUT: Sonar initialization
    pinMode(TRIGGER_PIN_FRONT, OUTPUT);
    pinMode(ECHO_PIN_FRONT, INPUT);
    pinMode(TRIGGER_PIN_LEFT, OUTPUT);
    pinMode(ECHO_PIN_LEFT, INPUT);
    pinMode(TRIGGER_PIN_RIGHT, OUTPUT);
    pinMode(ECHO_PIN_RIGHT, INPUT);
    */
    
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

/* COMMENTED OUT: Sonar reading function
float readSonar(int trigPin, int echoPin, float* readings) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2); 
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10); 
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH, 12000);
    float distance = (duration == 0) ? 200 : duration * 0.034 / 2;
    
    readings[readIndex] = distance;
    
    // Calculate median to filter out noise
    float sortedReadings[FILTER_SAMPLES]; 
    memcpy(sortedReadings, readings, sizeof(sortedReadings));
    for (int i = 0; i < FILTER_SAMPLES - 1; i++) {
        for (int j = i + 1; j < FILTER_SAMPLES; j++) {
            if (sortedReadings[j] < sortedReadings[i]) {
                float temp = sortedReadings[i];
                sortedReadings[i] = sortedReadings[j];
                sortedReadings[j] = temp;
            }
        }
    }
    
    float medianDistance = sortedReadings[FILTER_SAMPLES / 2];
    
    // --- Added stuck-check for left sensor ---
    if(trigPin == TRIGGER_PIN_LEFT) {
        static float lastValidLeft = medianDistance;      // Last reading that changed significantly.
        static unsigned long lastLeftChangeTime = millis(); // Last time the reading changed.
        const float TOLERANCE = 0.5;                        // Allowed variation (cm).
        const unsigned long STUCK_TIMEOUT = 4000;           // 4 seconds.
        
        // If the reading has changed by more than TOLERANCE, update lastValidLeft and reset timer.
        if (fabs(medianDistance - lastValidLeft) > TOLERANCE) {
            lastValidLeft = medianDistance;
            lastLeftChangeTime = millis();
        } else if (millis() - lastLeftChangeTime > STUCK_TIMEOUT) {
            // If the reading hasn't changed for 4 seconds, override it.
            medianDistance = 200; // Safe default (no obstacle).
        }
    }
    
    return medianDistance;
}
*/

// Update sensor readings (only LIDAR for front, sides set to max)
void updateSonarReadings() {
    if (millis() - lastSonarUpdate >= SONAR_UPDATE_INTERVAL) {
        // Read front LIDAR
        lastFrontDist = readLIDAR(frontReadings);
        
        // Set side distances to maximum (no obstacles)
        lastLeftDist = 500;
        lastRightDist = 500;
        
        /* COMMENTED OUT: Side sonar readings
        switch(currentSonar) {
            case 0:
                // Front already handled by LIDAR above
                currentSonar = 1;
                break;
            case 1:
                lastLeftDist = readSonar(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT, leftReadings);
                currentSonar = 2;
                break;
            case 2:
                lastRightDist = readSonar(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, rightReadings);
                currentSonar = 0;
                break;
        }
        */
        
        readIndex = (readIndex + 1) % FILTER_SAMPLES;
        lastSonarUpdate = millis();
    }
}

#endif // SONAR_H