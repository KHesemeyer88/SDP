#include "http_server.h"
#include "webpage.h"
#include "rtos_tasks.h"
#include "logging.h"
#include "websocket_handler.h"
#include "gnss.h"

// HTTP server instance on port 80
AsyncWebServer server(80);

// Initialize HTTP server with all route handlers
void initHttpServer() {
    
    // Root route - serves the main webpage
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", webPage);
    });
    
    // Fusion status endpoint
    // server.on("/fusionStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    //     char fusionStatus[64]; // Buffer for status
    //     char responseBuffer[256]; // Buffer for JSON response
    //     LOG_DEBUG("about to call getFusionStatus");
    //     // Get actual fusion status using our buffer-based function
    //     getFusionStatus(fusionStatus, sizeof(fusionStatus));
        
    //     // Format the JSON response
    //     snprintf(responseBuffer, sizeof(responseBuffer), 
    //              "{\"status\":\"%s\"}", fusionStatus);
        
    //     // Send response
    //     request->send(200, "application/json", responseBuffer);
    // });

    // Start the server
    server.begin();
    LOG_DEBUG("initHttpServer COMPLETE");
}

// HTTP server task function
void HttpServerTask(void *pvParameters) {
    // Initialize task
    LOG_DEBUG("HTTP task start");
    
    // Initialize server routes and start the server
    initHttpServer();
    
    // Task loop - AsyncWebServer doesn't need polling, but we keep
    // the task alive for future expansion and proper task management
    for (;;) {        
        // Task heartbeat
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}