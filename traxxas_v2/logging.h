#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// Log file path
#define SETUP_LOG_FILE "/setup_log.txt"

// Global file handle for logging
File logFile;
bool sdCardAvailable = false;

// Initialize the SD card and open log file
bool initLogging() {
  Serial.println("Initializing SD card for logging...");
  
  // Configure SPI for SD card
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  SPI.setFrequency(SD_FREQUENCY);
  
  // Initialize SD card
  if (!SD.begin(SD_CS_PIN, SPI, SD_FREQUENCY)) {
    Serial.println("SD Card initialization failed. Logging disabled.");
    return false;
  }
  
  Serial.println("SD Card initialized successfully for logging.");
  
  // Create a new setup log file, overwriting any existing file
  logFile = SD.open(SETUP_LOG_FILE, FILE_WRITE);
  if (!logFile) {
    Serial.println("Failed to open setup log file for writing.");
    return false;
  }
  
  // Write header to log file
  String header = "==== RC Car Setup Log ====\n";
  header += "Started: " + String(millis()) + "ms since boot\n";
  header += "=========================\n";
  
  logFile.print(header);
  logFile.flush();
  
  sdCardAvailable = true;
  return true;
}

// Log a message to SD card
void log(const String &message) {
  if (!sdCardAvailable || !logFile) return;
  
  // Write to SD card
  logFile.println(message);
  
  // Flush every time during setup to ensure data is written
  // (This is OK for setup since timing isn't critical)
  logFile.flush();
}

// Close log file properly
void closeLog() {
  if (sdCardAvailable && logFile) {
    logFile.println("==== End of setup log ====");
    logFile.flush();
    logFile.close();
  }
}

#endif // LOGGING_H