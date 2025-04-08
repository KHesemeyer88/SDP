// gnss.h - updated for full integration with ubx_parser
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "ubx_parser.h"  // provides UBX_NAV_PVT_data_t

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
extern volatile GNSSData gnssShadow;

extern volatile CorrectionStatus rtcmCorrectionStatus;
extern unsigned long correctionAge;

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

bool generateGGA(const UBX_NAV_PVT_data_t* pvt, char* out, size_t outLen);
void GGATask(void *pvParameters);
bool encodeBase64(const char* input, char* output, size_t outputSize);
extern SemaphoreHandle_t rtcmRingMutex;

extern SemaphoreHandle_t gnssSpiMutex;

#ifdef __cplusplus
}
#endif