#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "config.h" // For constants like WAYPOINT_REACHED_RADIUS

// External GPS variables
extern SFE_UBLOX_GNSS myGPS;
extern float targetLat, targetLon;
extern float waypointLats[], waypointLons[];
extern int waypointCount, currentWaypointIndex;
extern bool followingWaypoints;

// External pace control and tracking variables
extern bool autonomousMode;
extern unsigned long lastPaceUpdate;
extern float targetPace;
extern float currentPace;
extern float totalDistance;
extern float averagePace;
extern float lastTrackedLat;
extern float lastTrackedLon;
extern unsigned long lastDistanceUpdate;

extern unsigned long startTime;         // Set when autonomous mode starts
extern unsigned long finalElapsedTime;    // Set when autonomous mode stops

extern Servo escServo;

// store the latest GPS data
extern volatile float currentLat, currentLon;
extern volatile float currentSpeed;
extern volatile uint8_t currentFixType;
extern volatile bool newPVTDataAvailable;
extern volatile int carrSoln;
extern volatile double hAcc;

unsigned long lastReceivedRTCM_ms = 0;          //5 RTCM messages take approximately ~300ms to arrive at 115200bps
const unsigned long maxTimeBeforeHangup_ms = 10000UL; //If we fail to get a complete RTCM frame after 10s, then disconnect from caster
bool transmitLocation = true;
extern WiFiClient ntripClient;

extern volatile CorrectionStatus rtcmCorrectionStatus;
extern unsigned long correctionAge;

// Add this to the top of navigation.h
void pvtCallback(UBX_NAV_PVT_data_t *pvtData);

// Update the callback function implementation
void pvtCallback(UBX_NAV_PVT_data_t *pvtData)
{
    //Serial.println("PVT Callback called!");
    
    // Update global position variables with new data
    currentLat = pvtData->lat / 10000000.0;
    currentLon = pvtData->lon / 10000000.0;
    currentSpeed = pvtData->gSpeed / 1000.0;  // Convert from mm/s to m/s
    currentFixType = pvtData->fixType;
    
    carrSoln = pvtData->flags.bits.carrSoln;
    hAcc = pvtData->hAcc / 10.0; //cm
    // Set flag to indicate new data is available
    newPVTDataAvailable = true;
}

void pushGPGGA(NMEA_GGA_data_t *nmeaData) {
  //Provide the caster with our current position as needed
  if ((ntripClient.connected() == true) && (transmitLocation == true))
  {
    //Serial.print(F("Pushing GGA to server: "));
    //Serial.print((const char *)nmeaData->nmea); // .nmea is printable (NULL-terminated) and already has \r\n on the end

    //Push our current GGA sentence to caster
    ntripClient.print((const char *)nmeaData->nmea);
  }
}

// Initialize GPS module
bool initializeGPS() {
    Wire.begin(32, 33);
    if (!myGPS.begin()) {
        Serial.println("u-blox GPS not detected. Check wiring.");
        return false;
    }
    
    Serial.println("GPS module found!");
    
    //myGPS.setI2COutput(COM_TYPE_UBX);
    myGPS.setNavigationFrequency(NAV_FREQ);
    
    myGPS.setI2CInput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); //Be sure RTCM3 input is enabled. UBX + RTCM3 is not a valid state.
    myGPS.setDGNSSConfiguration(SFE_UBLOX_DGNSS_MODE_FIXED); // Set the differential mode - ambiguities are fixed whenever possible
    //myGNSS.setNavigationFrequency(20); //Set output in Hz.
      // Set the Main Talker ID to "GP". The NMEA GGA messages will be GPGGA instead of GNGGA
    myGPS.setMainTalkerID(SFE_UBLOX_MAIN_TALKER_ID_GP);
    myGPS.setNMEAGPGGAcallbackPtr(&pushGPGGA); // Set up the callback for GPGGA
    myGPS.setVal8(UBLOX_CFG_MSGOUT_NMEA_ID_GGA_I2C, 20); // Tell the module to output GGA every 10 seconds

    // Enable the callback for PVT messages
    myGPS.setAutoPVTcallbackPtr(&pvtCallback);
    
    // Set Dynamic Model to Robotic Lawn Mower (11) 
    // Also need to try: portable (0), pedestrian (3), automotive (4), wrist (9), bike(10), escooter (12)
    // if (!myGPS.setVal8(0x20110021, 11)) {
    //     Serial.println("Failed to set dynamic model to RLM.");
    // } else {
    //     Serial.println("Dynamic model set to Robotic Lawn Mower (RLM).");
    // }
    // Disable all sensor fusion: (does not seem to improve performance)
    if (!myGPS.setVal8(UBLOX_CFG_SFCORE_USE_SF, 0)) {
        Serial.println("Failed to disable sensor fusion.");
    } else {
        Serial.println("IMU sensor fusion disabled.");
    }
    
    return true;
}

// Get fusion status from IMU 
String getFusionStatus() {
    // Temporarily set nav frequency to 1Hz for status check
    myGPS.setNavigationFrequency(1);
    String fusionStatus = "Failed to get fusion data";

    // Try to get ESF info with timeout
    unsigned long startTimeLocal = millis();
    const unsigned long ESF_TIMEOUT = 1000; // 1 second timeout

    if (!myGPS.getEsfInfo()) {
        while (millis() - startTimeLocal < ESF_TIMEOUT) {
            if (myGPS.getEsfInfo()) {
                uint8_t fusionMode = myGPS.packetUBXESFSTATUS->data.fusionMode;
                switch(fusionMode) {
                    case 0: fusionStatus = "Initializing"; break;
                    case 1: fusionStatus = "Calibrated"; break;
                    case 2: fusionStatus = "Suspended"; break;
                    case 3: fusionStatus = "Disabled"; break;
                }
                break;
            }
        }
    }

    // Restore original navigation frequency
    myGPS.setNavigationFrequency(NAV_FREQ);
    return fusionStatus;
}

// Calculate distance between two coordinates using the Haversine formula
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    float lat1Rad = lat1 * PI / 180.0;
    float lon1Rad = lon1 * PI / 180.0;
    float lat2Rad = lat2 * PI / 180.0;
    float lon2Rad = lon2 * PI / 180.0;

    float dLat = lat2Rad - lat1Rad;
    float dLon = lon2Rad - lon1Rad;
    float a = sin(dLat/2) * sin(dLat/2) +
              cos(lat1Rad) * cos(lat2Rad) *
              sin(dLon/2) * sin(dLon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    return c * 6371000; // Earth radius in meters
}

// Calculate bearing between two coordinates
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    float lat1Rad = lat1 * PI / 180.0;
    float lon1Rad = lon1 * PI / 180.0;
    float lat2Rad = lat2 * PI / 180.0;
    float lon2Rad = lon2 * PI / 180.0;

    float dLon = lon2Rad - lon1Rad;
    float y = sin(dLon) * cos(lat2Rad);
    float x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(dLon);


    float bearing = atan2(y, x) * 180.0 / PI;
    return fmod((bearing + 360.0), 360.0);
}

// Get current GPS coordinates
void getCurrentPosition(float &lat, float &lon) {
    lat = currentLat;
    lon = currentLon;
}

// Record a waypoint at the current position
bool recordWaypoint() {
    if (waypointCount >= MAX_WAYPOINTS)
        return false; // Waypoint limit reached

    waypointLats[waypointCount] = currentLat;
    waypointLons[waypointCount] = currentLon;
    waypointCount++;

    return true;
}

// Clear all waypoints
void clearWaypoints() {
    waypointCount = 0;
}

// Calculate steering angle based on heading error (simple proportional control)
// In your navigation code where you calculate steering
float calculateSteeringAngle(float currentLat, float currentLon) {
    // Adjust proportional gain based on correction quality
    float correctionFactor = 0.35; // Default value
    
    if (rtcmCorrectionStatus == CORR_FRESH) {
        // Full confidence in position - use normal gain
        correctionFactor = 0.35;
    } else if (rtcmCorrectionStatus == CORR_STALE) {
        // Reduced confidence - use more conservative steering
        correctionFactor = 0.25;
    } else {
        // No corrections - use very conservative steering
        correctionFactor = 0.15;
    }
    
    // Your existing bearing/heading calculation
    float distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
    float bearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
    float currentHeading = myGPS.getHeading() / 100000.0;
    
    float headingError = bearing - currentHeading;
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;
    
    int steeringAngle = STEERING_CENTER + (headingError * correctionFactor);
    
    return steeringAngle;
}

// Only adjust speed if we have a target pace and no obstacle avoidance active
void updatePaceControl() {
    // Only process if in autonomous mode
    if (!autonomousMode) {
        return;
    }
    
    unsigned long currentTime = millis();
    
    // Only update at the specified interval
    if (currentTime - lastPaceUpdate <= SPEED_CORRECTION_INTERVAL) {
        return;
    }
    
    // Static variables for PI control
    static float cumulativeError = 0;
    static int lastEscCommand = ESC_MIN_FWD; // Track our last command instead of reading
    
    // Get current speed and update pace
    float speedMps = currentSpeed;
    currentPace = speedMps;
    
    // Calculate average pace
    unsigned long elapsedTime = currentTime - startTime;
    if (elapsedTime > 0) {
        averagePace = totalDistance / (elapsedTime / 1000.0);
    }
    
    // Only adjust speed if we have a target pace and no obstacle avoidance active
    if (targetPace > 0 && lastAvoidanceMessage == "") {
        float paceError = targetPace - currentPace;
        int adjustmentValue = 0;
        
        // Vehicle is stopped or moving very slowly (GPS noise)
        if (currentPace < 0.5) {
            // Initial push to overcome inertia - use a fixed starting value
            lastEscCommand = ESC_MIN_FWD + 15;
            cumulativeError = 0; // Reset integral term on fresh start
        }
        // Normal PI control operation when vehicle is moving
        else {
            // Proportional component
            float pComponent = paceError * 1.5;
            
            // Integral component with slower accumulation
            cumulativeError = constrain(cumulativeError + paceError * 0.03, -3.0, 5.0);
            
            // integral gain
            float iComponent = cumulativeError * 1.0;
            
            // Combined adjustment from PI components
            adjustmentValue = (int)(pComponent + iComponent);
            
            // Constrain adjustment to prevent lurching
            adjustmentValue = constrain(adjustmentValue, -1, 1);
            
            // Calculate new ESC value based on our previous command
            lastEscCommand = lastEscCommand + adjustmentValue;
        }
        
        // Minimum throttle floor based on target pace
        float minPowerLevel = ESC_MIN_FWD + (targetPace * 10.0);
        //minPowerLevel = constrain(minPowerLevel, ESC_MIN_FWD + 10, ESC_MIN_FWD + 25);
        
        if (currentPace > 0.5 && lastEscCommand < minPowerLevel) {
            lastEscCommand = (int)minPowerLevel;
        }
        
        // Apply and constrain final value
        lastEscCommand = constrain(lastEscCommand, ESC_MIN_FWD, ESC_MAX_FWD);
        escServo.write(lastEscCommand);
    }
    
    // Update the timestamp for next interval
    lastPaceUpdate = currentTime;
}

// Update distance tracking: add incremental distance only when GPS reports movement
void updateDistanceTracking() {
    if (millis() - lastDistanceUpdate >= 1000) {
        // Only update if we have valid previous coordinates
        if (lastTrackedLat != 0 && lastTrackedLon != 0 && currentLat != 0 && currentLon != 0) {
            float delta = calculateDistance(lastTrackedLat, lastTrackedLon, currentLat, currentLon);
            
            // Use a looser filter since we're checking speed as well
            if (delta < 10.0 && currentSpeed > 0.5) {
                totalDistance += delta;
            }
            
            if (targetDistance > 0 && totalDistance >= targetDistance) {
                autonomousMode = false;
                destinationReached = true;
                finalElapsedTime = millis() - startTime;
                lastAvoidanceMessage = "Target distance reached";
                escServo.write(ESC_NEUTRAL);
                return;
            }
        }
        
        // Update tracked position regardless of whether we calculated distance
        lastTrackedLat = currentLat;
        lastTrackedLon = currentLon;
        lastDistanceUpdate = millis();
    }
}

// Reset distance and time tracking for a new session
void resetTracking() {
    totalDistance = 0.0;
    startTime = (autonomousMode) ? millis() : 0;
    float currentLat, currentLon;
    getCurrentPosition(currentLat, currentLon);
    lastTrackedLat = currentLat;
    lastTrackedLon = currentLon;
}

// update obstacle avoidance and destination reached flash messages
void updateStatusMessages() {
  if (lastAvoidanceMessage != "") {
    if (lastAvoidanceMessage == "Destination reached" ||
        lastAvoidanceMessage == "Target distance reached") {
      if (millis() - destinationReachedTime > DESTINATION_MESSAGE_TIMEOUT) {
        lastAvoidanceMessage = "";
        destinationReached = false;
      }
    } else {
      if (millis() - lastAvoidanceTime > AVOIDANCE_MESSAGE_TIMEOUT) {
        lastAvoidanceMessage = "";
      }
    }
  }
}

// increment waypoint index, calculate segment distance, stop auto nav if distanced reached
void handleWaypointReached() {
    if (followingWaypoints) {
      if (currentWaypointIndex < waypointCount - 1) {
        currentWaypointIndex++;
      } else {
        currentWaypointIndex = 0;
        lastAvoidanceMessage = "Starting waypoint sequence again";
      }
      targetLat = waypointLats[currentWaypointIndex];
      targetLon = waypointLons[currentWaypointIndex];
      float currentLat, currentLon;
      getCurrentPosition(currentLat, currentLon);
      lastSegmentDistance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
    } else {
      autonomousMode = false;
      destinationReached = true;
      destinationReachedTime = millis();
      finalElapsedTime = millis() - startTime;  // Capture the final elapsed time
      lastAvoidanceMessage = "Destination reached";
      escServo.write(ESC_NEUTRAL);
    }
  }





//Check for the arrival of any correction data. Push it to the GPS.
//Return false if: the connection has dropped, or if we receive no data for maxTimeBeforeHangup_ms
bool processConnection() {
  if (!ntripClient.connected()) {
    return false;
  }
  
  if (ntripClient.available()) {
    // Create a buffer large enough to hold multiple RTCM messages
    uint8_t rtcmBuffer[512*4];
    size_t rtcmCount = 0;
    
    // Collect all available RTCM data
    while (ntripClient.available() && rtcmCount < sizeof(rtcmBuffer)) {
      rtcmBuffer[rtcmCount++] = ntripClient.read();
    }
    
    if (rtcmCount > 0) {
      // Update the timestamp for when we last received any correction data
      lastReceivedRTCM_ms = millis();
      
      // Push all collected data to the GPS module at once
      myGPS.pushRawData(rtcmBuffer, rtcmCount);
      
      Serial.print("Pushed ");
      Serial.print(rtcmCount);
      Serial.println(" RTCM bytes to GPS");
    }
  }
  
  // Check for timeout, but don't block - just report status
  if ((millis() - lastReceivedRTCM_ms) > maxTimeBeforeHangup_ms) {
    return false;
  }

  // Update correction status based on age
  correctionAge = millis() - lastReceivedRTCM_ms;
  if (correctionAge < 5000) { // Less than 5 seconds old
    rtcmCorrectionStatus = CORR_FRESH;
  } else if (correctionAge < 30000) { // Less than 30 seconds old
    rtcmCorrectionStatus = CORR_STALE;
  } else {
    rtcmCorrectionStatus = CORR_NONE;
  }
  
  return true;
}

bool connectToNTRIP() {
  if (!ntripClient.connected()) {
    Serial.print("Connecting to NTRIP caster: ");
    Serial.println(casterHost);
    
    if (!ntripClient.connect(casterHost, casterPort)) {
      Serial.println("Connection to caster failed");
      return false;
    }
    
    // Formulate the NTRIP request
    char serverRequest[512];
    snprintf(serverRequest, sizeof(serverRequest),
           "GET /%s HTTP/1.0\r\n"
           "User-Agent: NTRIP SparkFun u-blox Client v1.0\r\n"
           "Accept: */*\r\n"
           "Connection: close\r\n"
           "Authorization: Basic %s\r\n"
           "\r\n",
           mountPoint, base64::encode(String(casterUser) + ":" + String(casterUserPW)).c_str());
    
    ntripClient.write(serverRequest, strlen(serverRequest));
    
    // Wait up to 5 seconds for response
    unsigned long startTimeLocal = millis();
    while (ntripClient.available() == 0) {
      if (millis() > (startTimeLocal + 5000)) {
        Serial.println("Caster timed out!");
        ntripClient.stop();
        return false;
      }
      delay(10);
    }
    
    // Check reply
    int connectionResult = 0;
    char response[512];
    size_t responseSpot = 0;
    
    while (ntripClient.available()) { // Read bytes from the caster and store them
      if (responseSpot == sizeof(response) - 1) // Exit the loop if we get too much data
        break;
        
      response[responseSpot++] = ntripClient.read();
      
      if (connectionResult == 0) { // Only print success/fail once
        if (strstr(response, "200") != nullptr) { //Look for '200 OK'
          connectionResult = 200;
        }
        if (strstr(response, "401") != nullptr) { //Look for '401 Unauthorized'
          Serial.println("Hey - your credentials look bad! Check your caster username and password.");
          connectionResult = 401;
        }
      }
    }
    
    response[responseSpot] = '\0'; // NULL-terminate the response
    
    if (connectionResult != 200) {
      Serial.print("Failed to connect to ");
      Serial.println(casterHost);
      return false;
    } else {
      Serial.print("Connected to: ");
      Serial.println(casterHost);
      lastReceivedRTCM_ms = millis(); // Reset timeout
      return true;
    }
  }
  
  return true; // Already connected
}

#endif // NAVIGATION_H