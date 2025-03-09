/* left sonar reading is off. If the reading stucks at the same value for more than 4seconds, gives it max reading (200)  */ 

#ifndef SONAR_H
#define SONAR_H

#include <Arduino.h>
#include "config.h" 

// Sonar filter arrays
extern float frontReadings[];
extern float leftReadings[];
extern float rightReadings[];
extern int readIndex;

// Sonar measurements
extern float lastFrontDist, lastLeftDist, lastRightDist;
extern unsigned long lastSonarUpdate;
extern int currentSonar;

// Initialize sonar sensors
void initializeSonar() {
    pinMode(TRIGGER_PIN_FRONT, OUTPUT);
    pinMode(ECHO_PIN_FRONT, INPUT);
    pinMode(TRIGGER_PIN_LEFT, OUTPUT);
    pinMode(ECHO_PIN_LEFT, INPUT);
    pinMode(TRIGGER_PIN_RIGHT, OUTPUT);
    pinMode(ECHO_PIN_RIGHT, INPUT);
    
    // Initialize reading arrays to maximum distance
    for (int i = 0; i < FILTER_SAMPLES; i++) {
        frontReadings[i] = 200;
        leftReadings[i] = 200;
        rightReadings[i] = 200;
    }
}

// Read distance from a single sonar sensor
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

// Update readings from all sonar sensors (called from loop)
void updateSonarReadings() {
    if (millis() - lastSonarUpdate >= SONAR_UPDATE_INTERVAL) {
        switch(currentSonar) {
            case 0: 
                lastFrontDist = readSonar(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, frontReadings);
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
        
        readIndex = (readIndex + 1) % FILTER_SAMPLES;
        lastSonarUpdate = millis();
    }
}

#endif // SONAR_H
