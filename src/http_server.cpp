#include "http_server.h"
#include "webpage.h"
#include "rtos_tasks.h"
#include "logging.h"
#include "websocket_handler.h"

// HTTP server instance on port 80
AsyncWebServer server(80);

// Initialize HTTP server with all route handlers
void initHttpServer() {
    
    // Root route - serves the main webpage
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", webPage);
    });
    
    // Fusion status endpoint
    server.on("/fusionStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
        String response = "{\"status\":\"ESF disabled\"}";
        request->send(200, "application/json", response);
    });
    
    // Start the server
    server.begin();
    LOG_DEBUG("HTTP server initialized and ready");
}

// HTTP server task function
void HttpServerTask(void *pvParameters) {
    // Initialize task
    LOG_DEBUG("HTTP Server Task Started");
    
    // Initialize server routes and start the server
    initHttpServer();
    
    // Task loop - AsyncWebServer doesn't need polling, but we keep
    // the task alive for future expansion and proper task management
    for (;;) {        
        // Task heartbeat
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}