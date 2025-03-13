#ifndef WEBHANDLERS_H
#define WEBHANDLERS_H

#include <WebServer.h>
#include "config.h"
#include "webpage.h"
#include "navigation.h"

// Forward declarations
extern WebServer server;
extern SFE_UBLOX_GNSS myGPS;

// Function to set up web server routes
void setupWebServerRoutes() {
    // Serve the main webpage
    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", webPage);
    });

    // Fusion status endpoint - retained as HTTP since it's infrequently used
    // and requires special handling on the ESP32
    server.on("/fusionStatus", HTTP_GET, []() {
        myGPS.setNavigationFrequency(1);
        String fusionStatus = "Failed to get fusion data";
        
        if (!myGPS.getEsfInfo()) {
            unsigned long startTime = millis();
            unsigned long timeout = 2000; // 2 second timeout
            
            while(millis() - startTime < timeout) {
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

        String json = "{\"status\":\"" + fusionStatus + "\"}";
        server.send(200, "application/json", json);
        myGPS.setNavigationFrequency(NAV_FREQ);
    });
}

#endif // WEBHANDLERS_H