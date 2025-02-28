// sonar.h
#ifndef SONAR_H
#define SONAR_H

#include <Arduino.h>
#include "config.h" // For pin definitions and constants

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
    delayMicroseconds(2); // per HC-SR04 specs
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10); // per HC-SR04 specs
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH, 12000); // measure time to receive response, wait up to 12ms (about 2m)
    float distance = (duration == 0) ? 200 : duration * 0.034 / 2; //speed of sound; return max distance if no echo
    
    // Update array if we got a valid reading
    readings[readIndex] = distance;
    
    // Calculate median to filter out false positives
    float sortedReadings[FILTER_SAMPLES]; 
    memcpy(sortedReadings, readings, sizeof(sortedReadings));
    for(int i = 0; i < FILTER_SAMPLES-1; i++) {
        for(int j = i + 1; j < FILTER_SAMPLES; j++) {
            if(sortedReadings[j] < sortedReadings[i]) {
                float temp = sortedReadings[i];
                sortedReadings[i] = sortedReadings[j];
                sortedReadings[j] = temp;
            }
        }
    }
    
    return sortedReadings[FILTER_SAMPLES/2];  // Return median value
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