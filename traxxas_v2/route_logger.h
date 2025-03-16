#ifndef ROUTE_LOGGER_H
#define ROUTE_LOGGER_H

#include <Arduino.h>
#include <SD.h>
#include "config.h"

// Directory and file paths
#define ROUTES_DIR "/routes"
#define DEFAULT_ROUTE "/routes/test_route.csv"

// Global control flag to enable route logging
bool routeLoggingEnabled = true;

// Initialize the routes directory
bool initRouteLogging() {
  if (!routeLoggingEnabled) return false;
  
  // Check if SD card is available from previous initialization
  if (!SD.begin(SD_CS_PIN)) {
    return false;
  }
  
  // Create routes directory if it doesn't exist
  if (!SD.exists(ROUTES_DIR)) {
    if (!SD.mkdir(ROUTES_DIR)) {
      return false;
    }
  }
  
  // Create or clear the default route file
  File routeFile = SD.open(DEFAULT_ROUTE, FILE_WRITE);
  if (!routeFile) {
    return false;
  }
  
  // Write header line
  routeFile.println("latitude,longitude,rtk_status,fix_type");
  routeFile.close();
  
  return true;
}

// Enable or disable route logging
void setRouteLoggingEnabled(bool enabled) {
  routeLoggingEnabled = enabled;
}

// Record a waypoint in the route file
bool recordRouteWaypoint(float latitude, float longitude, int rtkStatus, int fixType, const char* routeName = NULL) {
  if (!routeLoggingEnabled) return false;
  
  String filePath;
  
  // Use default route name if none provided
  if (routeName == NULL) {
    filePath = DEFAULT_ROUTE;
  } else {
    filePath = String(ROUTES_DIR) + "/" + String(routeName) + ".csv";
  }
  
  // Open route file for appending
  File routeFile = SD.open(filePath, FILE_APPEND);
  if (!routeFile) {
    return false;
  }
  
  // Build the CSV line with the waypoint data
  String waypointLine = String(latitude, 8) + "," + 
                        String(longitude, 8) + "," + 
                        String(rtkStatus) + "," + 
                        String(fixType);
  
  // Write the waypoint data
  routeFile.println(waypointLine);
  routeFile.close();
  
  return true;
}

// Load waypoints from a saved route
int loadRouteWaypoints(float* latitudes, float* longitudes, int maxWaypoints, const char* routeName = NULL) {
  if (!routeLoggingEnabled) return 0;
  
  String filePath;
  
  // Use default route name if none provided
  if (routeName == NULL) {
    filePath = DEFAULT_ROUTE;
  } else {
    filePath = String(ROUTES_DIR) + "/" + String(routeName) + ".csv";
  }
  
  // Check if route file exists
  if (!SD.exists(filePath)) {
    return 0;
  }
  
  // Open route file for reading
  File routeFile = SD.open(filePath);
  if (!routeFile) {
    return 0;
  }
  
  // Skip header line
  String headerLine = routeFile.readStringUntil('\n');
  
  // Read waypoints
  int waypointCount = 0;
  while (routeFile.available() && waypointCount < maxWaypoints) {
    String line = routeFile.readStringUntil('\n');
    
    // Parse CSV line
    int commaIndex1 = line.indexOf(',');
    int commaIndex2 = line.indexOf(',', commaIndex1 + 1);
    int commaIndex3 = line.indexOf(',', commaIndex2 + 1);
    
    if (commaIndex1 > 0 && commaIndex2 > 0) {
      String latStr = line.substring(0, commaIndex1);
      String lonStr = line.substring(commaIndex1 + 1, commaIndex2);
      String rtkStr = line.substring(commaIndex2 + 1, commaIndex3);
      String fixStr = line.substring(commaIndex3 + 1);
      
      float lat = latStr.toFloat();
      float lon = lonStr.toFloat();
      int rtk = rtkStr.toInt();
      int fix = fixStr.toInt();
      
      // Only store waypoints that had an adequate fix
      if (fix >= 3) {  // Require at least a 3D fix
        latitudes[waypointCount] = lat;
        longitudes[waypointCount] = lon;
        waypointCount++;
      }
    }
  }
  
  routeFile.close();
  return waypointCount;
}

// Reset the route file (create a new empty one)
bool resetRouteFile(const char* routeName = NULL) {
  if (!routeLoggingEnabled) return false;
  
  String filePath;
  
  // Use default route name if none provided
  if (routeName == NULL) {
    filePath = DEFAULT_ROUTE;
  } else {
    filePath = String(ROUTES_DIR) + "/" + String(routeName) + ".csv";
  }
  
  // Delete existing file if it exists
  if (SD.exists(filePath)) {
    SD.remove(filePath);
  }
  
  // Create a new file with header
  File routeFile = SD.open(filePath, FILE_WRITE);
  if (!routeFile) {
    return false;
  }
  
  routeFile.println("latitude,longitude,rtk_status,fix_type");
  routeFile.close();
  
  return true;
}

#endif // ROUTE_LOGGER_H