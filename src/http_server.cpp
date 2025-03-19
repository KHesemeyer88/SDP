#include "http_server.h"
#include "webpage.h"
#include "rtos_tasks.h"

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
        // Sample response - update with actual data when implemented
        String response = "{\"status\":\"ESF disabled\"}";
        request->send(200, "application/json", response);
    });
    
    // Any other routes can be added here
    
    // Start the server
    server.begin();
    Serial.println("HTTP server initialized and ready to start");
}

// HTTP server task function
void HttpServerTask(void *pvParameters) {
    // Initialize task
    Serial.println("HTTP Server Task Started");
    
    // Initialize server routes and start the server
    initHttpServer();
    
    // Task loop - even though AsyncWebServer doesn't need polling,
    // we keep the task running to maintain proper task priority
    for (;;) {
        // Task heartbeat - prevents watchdog timeout
        // ESPAsyncWebServer handles requests through interrupts,
        // so we don't need to call any handler function in the loop
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}