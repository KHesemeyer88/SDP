#ifndef GNSS_H
#define GNSS_H

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Task priorities and stack sizes
#define GNSS_TASK_PRIORITY       9  // Lower than control but higher than websocket
#define GNSS_TASK_STACK_SIZE     8192

// GNSS task handle
extern TaskHandle_t gnssTaskHandle;

// Mutex for accessing GNSS data
extern SemaphoreHandle_t gnssMutex;

// GNSS object
extern SFE_UBLOX_GNSS myGPS;

// GNSS data structure to be shared between tasks
struct GNSSData {
    float latitude;
    float longitude;
    float speed;
    uint8_t fixType;
    int carrSoln;
    double hAcc;
    bool newDataAvailable;
};

extern volatile GNSSData gnssData;

// RTK Correction status tracking
extern volatile CorrectionStatus rtcmCorrectionStatus;
extern unsigned long correctionAge;
extern unsigned long lastReceivedRTCM_ms;

// Function declarations
void GNSSTask(void *pvParameters);
bool initializeGNSS();
void pvtCallback(UBX_NAV_PVT_data_t *pvtData);
void pushGPGGA(NMEA_GGA_data_t *nmeaData);
bool processGNSSConnection();
bool connectToNTRIP();
String getFusionStatus();

#endif // GNSS_H