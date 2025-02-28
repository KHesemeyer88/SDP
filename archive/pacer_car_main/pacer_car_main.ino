/*
  This example shows how to query a u-blox module for its position, velocity and time (PVT) data.
  We also turn off the NMEA output on the I2C port. This decreases the amount of I2C traffic dramatically.
  Note: Lat/Lon are large numbers because they are * 10^7. To convert lat/lon
  to something google maps understands simply divide the numbers by 10,000,000.

  examples used
  - sparkfun redboard
      - SD -> SD test & https://docs.arduino.cc/learn/programming/sd-guide/
  - GNSS v3
    - PVT
*/
/*#include <SparkFun_u-blox_GNSS_v3.h>
#include <sfe_bus.h>
#include <u-blox_Class_and_ID.h>
#include <u-blox_GNSS.h>
#include <u-blox_config_keys.h>
#include <u-blox_external_typedefs.h>
#include <u-blox_structs.h>


dead reckoning example6
SparkFun_u-blox_GNSS_v3/Adding_New_messages.md
speedheadingposistion ex

calibration stuff
u-blox_GNSS.h
example1 GetVal
example3 GetPortSettings
example power off
DATA LOGGING EXAMPLES

refresh rate
example16 nanosecond maxoutput
output rate examples
myGNSS.setNavigationFrequency(5); //Set output to 5 times a second
byte rate = myGNSS.getNavigationFrequency(); //Get the update rate of this module
Serial.print("Current update rate: ");
Serial.println(rate);
*/


#include <Wire.h> //Needed for I2C to GNSS
#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
#include "FS.h" // SD card
#include "SD.h" // SD card
#include "SPI.h"// SD card
SFE_UBLOX_GNSS myGNSS; // SFE_UBLOX_GNSS uses I2C. For Serial or SPI, see Example2 and Example3


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Wire.begin(); // Start I2C
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }
  while (myGNSS.begin() == false) { //Connect to the u-blox module using Wire port
    Serial.println(F("u-blox GNSS not detected at default I2C address. Retrying..."));
    delay (1000);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  //myGNSS.setESFAutoAlignment(true); //Enable Automatic IMU-mount Alignment from dead reckoning example4

  // Check IMU is calibrated
  if (myGNSS.getEsfInfo()){
    Serial.print(F("Fusion Mode: "));
    Serial.println(myGNSS.packetUBXESFSTATUS->data.fusionMode);  
    if (myGNSS.packetUBXESFSTATUS->data.fusionMode == 1){
      Serial.println(F("Fusion Mode is Initialized!"));  
		}
		else {
      Serial.println(F("Fusion Mode is either disabled or not initialized!"));  
		}
  }

  // ----- Save IMU calibration -----
  //myGNSS.set
  
  //myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Optional: save (only) the communications port settings to flash and BBR

  //const char *path = "/gnss.txt";
}

void loop() {

  // ----- Get coordinates -----
  if (myGNSS.getPVT()) {
    // Request (poll) the position, velocity and time (PVT) information.
    // The module only responds when a new position is available. Default is once per second.
    // getPVT() returns true when new data is received.
    int32_t latitude = myGNSS.getLatitude();
    int32_t longitude = myGNSS.getLongitude();
    
    long speed = myGNSS.getGroundSpeed();
    // Serial.print(F(" Speed: "));
    // Serial.print(speed);
    // Serial.print(F(" (mm/s)"));

    long heading = myGNSS.getHeading();
    // Serial.print(F(" Heading: "));
    // Serial.print(heading);
    // Serial.print(F(" (degrees * 10^-5)"));


    // write to Serial
    Serial.print(latitude / 10000000.0, 7);
    Serial.print(" ");
    Serial.print(longitude / 10000000.0, 7);
    Serial.print(" ");
    // write to SD
    // writeDoubleToFile(SD, "/gnss.txt", latitude / 10000000.0, 7);
    // appendFile(SD, "/gnss.txt", " ");
    // appendDoubleToFile(SD, "/gnss.txt", longitude / 10000000.0, 7);
    // appendFile(SD, "/gnss.txt", " ");


    // ----- IMU heading -----
    // ESF data is produced at the navigation rate, so by default we'll get fresh data once per second (was a delay(250))?  }
    if (myGNSS.getNAVATT()) {// Poll new NAV ATT data
      //Serial.print(F(" Heading: ")); 
      //Serial.println(myGNSS.getATTheading(), 2); // Use the helper function to get the heading in degrees  

      // write to Serial
      Serial.print(myGNSS.getATTheading(), 5);
      Serial.print(" ");

      // write to SD  
      // appendDoubleToFile(SD, "/gnss.txt", myGNSS.getATTheading(), 5);
      // appendFile(SD, "/gnss.txt", " ");
    }
  
    //worse heading appendDoubleToFile(SD, "/gnss.txt", heading / 100000.0, 5);
    //appendFile(SD, "/gnss.txt", " ");
    //appendDoubleToFile(SD, "/gnss.txt", speed / 1000.0, 3);
    Serial.println(speed / 1000.0, 3);
    
  }

    //readFile(SD, "/gnss.txt");
    //Serial.println("");
}

void readFile(fs::FS &fs, const char *path) {
  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}
void writeFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (!file.print(message)) {
    Serial.println("Write failed");
  }
  file.close();
}
void writeDoubleToFile(fs::FS &fs, const char *path, double value, int decimals) {
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (!file.print(value, decimals)) {
    Serial.println("Write failed");
  }
  file.close();
}
void appendFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (!file.print(message)) {
    Serial.println("Append failed");
  }
  file.close();
}
void appendDoubleToFile(fs::FS &fs, const char *path, double value, int decimals) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (!file.print(value, decimals)) {
    Serial.println("Append failed");
  }
  file.close();
}
void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}