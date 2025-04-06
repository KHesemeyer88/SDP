// gnss.h - rewritten to match custom UBX parser architecture
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Shared GNSS data structure
typedef struct {
    double latitude;
    double longitude;
    double speed;
    double heading;
    float hAcc;
    uint8_t fixType;
    uint8_t carrSoln;
    bool newDataAvailable;
    unsigned long gnssFixTime;
} GNSSData;

typedef enum CorrectionStatus CorrectionStatus;

// External handles
extern TaskHandle_t gnssTaskHandle;
extern SemaphoreHandle_t gnssMutex;
extern volatile GNSSData gnssData;
extern volatile CorrectionStatus rtcmCorrectionStatus;

// Setup GNSS hardware and parser
bool initializeGNSS();

// Poll GNSS for new data (call inside GNSSTask)
void processGNSSInput();

// Pull RTCM corrections from NTRIP client
bool processRTKConnection();

#ifdef __cplusplus
extern "C" {
#endif

void GNSSTask(void *pvParameters);

#ifdef __cplusplus
}
#endif

extern unsigned long correctionAge;
