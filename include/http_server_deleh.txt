#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

#include <ESPAsyncWebServer.h>
#include "config.h"

// HTTP server instance
extern AsyncWebServer server;

// Initialize HTTP server and WebSocket handlers
void initHttpServer();

// HTTP server task function
void HttpServerTask(void *pvParameters);

#endif // HTTP_SERVER_H