#ifndef SONAR_H
#define SONAR_H

#include <Arduino.h>
#include "config.h" 
#include "LIDARLite_v4LED.h"

// Sonar filter arrays
extern float frontReadings[];
extern float leftReadings[];
extern float rightReadings[];
extern int readIndex;

// Sonar/LIDAR measurements
extern float lastFrontDist, lastLeftDist, lastRightDist;
extern unsigned long lastSonarUpdate;
extern int currentSonar;

// LIDAR sensor object
LIDARLite_v4LED myLIDAR;

// Initialize sensors - now handles both LIDAR and sonar
void initializeSensors() {
    // Initialize LIDAR for front distance measurements
    bool lidarInitialized = false;
    
    // Attempt to initialize LIDAR sensor
    Wire.begin(LIDAR_SDA_PIN, LIDAR_SCL_PIN);
    Serial.println("Initializing LIDAR sensor...");
    
    if (myLIDAR.begin()) {
        Serial.println("LIDAR sensor initialized successfully!");
        lidarInitialized = true;
    } else {
        Serial.println("LIDAR initialization failed! Check wiring.");
        // Could implement a fallback to sonar here if needed
    }
    
    // Initialize sonar pins for left/right (currently disabled)
    // Commented out but left for future expansion
    /*
    pinMode(TRIGGER_PIN_LEFT, OUTPUT);
    pinMode(ECHO_PIN_LEFT, INPUT);
    pinMode(TRIGGER_PIN_RIGHT, OUTPUT);
    pinMode(ECHO_PIN_RIGHT, INPUT);
    */
    
    // Initialize reading arrays to maximum distance
    for (int i = 0; i < FILTER_SAMPLES; i++) {
        frontReadings[i] = 500;  // Max distance (500cm = 5m)
        leftReadings[i] = 500;
        rightReadings[i] = 500;
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

// Legacy function for sonar reading - kept for compatibility
// float readSonar(int trigPin, int echoPin, float* readings) {
//     digitalWrite(trigPin, LOW);
//     delayMicroseconds(2); 
//     digitalWrite(trigPin, HIGH);
//     delayMicroseconds(10); 
//     digitalWrite(trigPin, LOW);
    
//     long duration = pulseIn(echoPin, HIGH, 30000); // Increased timeout for longer distances
//     float distance = (duration == 0) ? 500 : duration * 0.034 / 2;
    
//     readings[readIndex] = distance;
    
//     // Calculate median to filter out noise
//     float sortedReadings[FILTER_SAMPLES]; 
//     memcpy(sortedReadings, readings, sizeof(sortedReadings));
//     for (int i = 0; i < FILTER_SAMPLES - 1; i++) {
//         for (int j = i + 1; j < FILTER_SAMPLES; j++) {
//             if (sortedReadings[j] < sortedReadings[i]) {
//                 float temp = sortedReadings[i];
//                 sortedReadings[i] = sortedReadings[j];
//                 sortedReadings[j] = temp;
//             }
//         }
//     }
    
//     float medianDistance = sortedReadings[FILTER_SAMPLES / 2];
    
//     // --- Kept stuck-check for left sensor ---
//     if(trigPin == TRIGGER_PIN_LEFT) {
//         static float lastValidLeft = medianDistance;      // Last reading that changed significantly.
//         static unsigned long lastLeftChangeTime = millis(); // Last time the reading changed.
//         const float TOLERANCE = 0.5;                        // Allowed variation (cm).
//         const unsigned long STUCK_TIMEOUT = 4000;           // 4 seconds.
        
//         // If the reading has changed by more than TOLERANCE, update lastValidLeft and reset timer.
//         if (fabs(medianDistance - lastValidLeft) > TOLERANCE) {
//             lastValidLeft = medianDistance;
//             lastLeftChangeTime = millis();
//         } else if (millis() - lastLeftChangeTime > STUCK_TIMEOUT) {
//             // If the reading hasn't changed for 4 seconds, override it.
//             medianDistance = 500; // Safe default (no obstacle, max distance).
//         }
//     }
    
//     return medianDistance;
// }

// Update readings from all distance sensors (called from loop)
void updateDistanceReadings() {
    if (millis() - lastSonarUpdate >= SONAR_UPDATE_INTERVAL) {
        // Update the front sensor using LIDAR
        lastFrontDist = readLIDAR(frontReadings);
        
        // Left and right sensors are commented out but code kept for future use
        /*
        switch(currentSonar) {
            case 0: 
                // Already handled front LIDAR above
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
        
        // For now, set left and right distances to max (no obstacle)
        lastLeftDist = 500;
        lastRightDist = 500;
        
        // Update index for the circular buffer
        readIndex = (readIndex + 1) % FILTER_SAMPLES;
        lastSonarUpdate = millis();
    }
}

#endif // SONAR_H