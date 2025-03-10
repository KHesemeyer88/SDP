#include <WiFi.h>
#include <AsyncTCP.h> // https://randomnerdtutorials.com/esp32-websocket-server-arduino/
#include <ESPAsyncWebServer.h> // download off git -> sketch include library .zip
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "base64.h"  // Install "ArduinoBase64" library
#include "credentials.h"
// #include <WiFiClient.h>// syronous stuff #include <WebServer.h> // #include <WebSocketsServer.h> // WebServer server(80); // WebSocketsServer webSocket(81);

WiFiClient ntripClient;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
SFE_UBLOX_GNSS myGNSS;

volatile double lat_current, lon_current, lat_phone, lon_phone;


double lat_waypoint, lon_waypoint;
bool atWaypoint = false;
float distance_to_waypoint;
double waypoint_tolerance = 10; //cm



int enable_RTK = 0;
int incrementor = 0;
int counter = 0;

volatile int NTRIP_conn = 0;
volatile int RTK_fix = 0;
volatile int RTK_carrier = 0;
volatile double horizontal_accuracy = 0;

unsigned long lastReceivedRTCM_ms = 0;          //5 RTCM messages take approximately ~300ms to arrive at 115200bps
const unsigned long maxTimeBeforeHangup_ms = 10000UL; //If we fail to get a complete RTCM frame after 10s, then disconnect from caster
bool transmitLocation = true;


float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    float lat1Rad = lat1 * PI / 180.0;
    float lon1Rad = lon1 * PI / 180.0;
    float lat2Rad = lat2 * PI / 180.0;
    float lon2Rad = lon2 * PI / 180.0;

    float dLat = lat2Rad - lat1Rad;
    float dLon = lon2Rad - lon1Rad;
    float a = sin(dLat/2) * sin(dLat/2) +
              cos(lat1Rad) * cos(lat2Rad) *
              sin(dLon/2) * sin(dLon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    return c * 6371000; // Earth radius in meters
}

void pushGPGGA(NMEA_GGA_data_t *nmeaData) {
  //Provide the caster with our current position as needed
  if ((ntripClient.connected() == true) && (transmitLocation == true))
  {
    Serial.print(F("Pushing GGA to server: "));
    Serial.print((const char *)nmeaData->nmea); // .nmea is printable (NULL-terminated) and already has \r\n on the end

    //Push our current GGA sentence to caster
    ntripClient.print((const char *)nmeaData->nmea);
  }
}

void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct) {
  lat_current = ubxDataStruct->lat / 10000000.0;
  // Serial.print(F("Lat: "));
  // Serial.print(lat_current / 10000000.0, 7);
  lon_current = ubxDataStruct->lon / 10000000.0;
  // Serial.print(F("  Long: "));
  // Serial.print(lon_current / 10000000.0, 7);

  // double altitude = ubxDataStruct->hMSL; // Print the height above mean sea level
  // Serial.print(F("  Height: "));
  // Serial.print(altitude / 1000.0, 3);

  uint8_t fixType = ubxDataStruct->fixType;
  RTK_fix = fixType;
  // Serial.print(F("  Fix: "));
  // Serial.print(fixType);
  // if (fixType == 0)
  //   Serial.print(F(" (None)"));
  // else if (fixType == 1)
  //   Serial.print(F(" (Dead Reckoning)"));
  // else if (fixType == 2)
  //   Serial.print(F(" (2D)"));
  // else if (fixType == 3)
  //   Serial.print(F(" (3D)"));
  // else if (fixType == 4)
  //   Serial.print(F(" (GNSS + Dead Reckoning)"));
  // else if (fixType == 5)
  //   Serial.print(F(" (Time Only)"));
  // else
  //   Serial.print(F(" (UNKNOWN)"));

  uint8_t carrSoln = ubxDataStruct->flags.bits.carrSoln;
  RTK_carrier = carrSoln;
  // Serial.print(F("  Carrier Solution: "));
  // Serial.print(carrSoln);
  // if (carrSoln == 0)
  //   Serial.print(F(" (None)"));
  // else if (carrSoln == 1)
  //   Serial.print(F(" (Floating)"));
  // else if (carrSoln == 2)
  //   Serial.print(F(" (Fixed)"));
  // else
  //   Serial.print(F(" (UNKNOWN)"));


  uint32_t hAcc = ubxDataStruct->hAcc; // Print the horizontal accuracy estimate
  horizontal_accuracy = hAcc / 1000.0;
  // Serial.print(F("  Horizontal Accuracy Estimate: "));
  // Serial.print(hAcc / 1000.0);
  // Serial.print(F(" (m)"));

  // Serial.println();    
}

// Callback: printRTCMdata1005 will be called when new RTCM 1005 data has been parsed from pushRawData
void printRTCMdata1005(RTCM_1005_data_t *rtcmData1005) {
  double x = rtcmData1005->AntennaReferencePointECEFX;
  x /= 10000.0; // Convert to m
  double y = rtcmData1005->AntennaReferencePointECEFY;
  y /= 10000.0; // Convert to m
  double z = rtcmData1005->AntennaReferencePointECEFZ;
  z /= 10000.0; // Convert to m

  Serial.print(F("NTRIP Server RTCM 1005:  ARP ECEF-X: "));
  Serial.print(x, 4); // 4 decimal places
  Serial.print(F("  Y: "));
  Serial.print(y, 4); // 4 decimal places
  Serial.print(F("  Z: "));
  Serial.println(z, 4); // 4 decimal places
}
// Callback: printRTCMdata1006 will be called when new RTCM 1006 data has been parsed from pushRawData
void printRTCMdata1006(RTCM_1006_data_t *rtcmData1006) {
  double x = rtcmData1006->AntennaReferencePointECEFX;
  x /= 10000.0; // Convert to m
  double y = rtcmData1006->AntennaReferencePointECEFY;
  y /= 10000.0; // Convert to m
  double z = rtcmData1006->AntennaReferencePointECEFZ;
  z /= 10000.0; // Convert to m
  double h = rtcmData1006->AntennaHeight;
  h /= 10000.0; // Convert to m

  Serial.print(F("NTRIP Server RTCM 1006:  ARP ECEF-X: "));
  Serial.print(x, 4); // 4 decimal places
  Serial.print(F("  Y: "));
  Serial.print(y, 4); // 4 decimal places
  Serial.print(F("  Z: "));
  Serial.print(z, 4); // 4 decimal places
  Serial.print(F("  Height: "));
  Serial.println(h, 4); // 4 decimal places
}
/*
        function sendInput() {
            let inputVal = document.getElementById('inputBox').value;
            socket.send('input:' + inputVal);
        }
  <body onload="init()">
  body<div>
    <input id="inputText" type="text" placeholder="Type a message">
    <button onclick="sendMessage()">Send</button>
  </div>
    <script>
        ws.onmessage = function(event) {
            document.getElementById("status").innerText = event.data;
        };
        function sendMessage() {
            var message = document.getElementById("inputText").value;
            ws.send(message);
        }
    </script>
*/


const char* myWebpage PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>CDR RTK</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
</head>
<body>
  <div>
    <h1>CDR RTK</h1>

    <p>Current lat: <span id="lat">lat</span></p>
    <p>Current lon: <span id="lon">lon</span></p>
    <!-- <p>phone lat: <span id="lat_phone"></span></p>
    <p>phone lon: <span id="lon_phone"></span></p> -->
    <button id="RTK_indicator" onclick="toggle_RTK()">RTK is disabled</button>

    <!-- <button onclick="increment()">Increment Counter <span id="counter">0</span></button>
    <p>incrementor check: <span id="incrementor">0</span></p>
    <button onclick="requestLocation()">Send Location</button> -->

    <p>NTRIP: <span id="NTRIP_conn"></span></p>
    <p>Fix Type: <span id="RTK_fix"></span></p>
    <p>RTK carrier: <span id="RTK_carrier"></span></p>
    <p>horizontal accuracy estimate: <span id="horizontal_accuracy"></span></p>

    <br></br>
    <button id="waypoint_button" onclick="recordWaypoint()">Record Waypoint</button>
    <p>Dist to waypoint: <span id="dist">Unkown</span></p>
  </div>

  <script>
    var ws;
    function init() {
      ws = new WebSocket("ws://" + window.location.host + "/ws");
      ws.onopen = function() {
        alert("Connected");
        // if (navigator.geolocation) {
        //     //alert("entered");
        //     navigator.geolocation.getCurrentPosition(sendLocation, error, {
                    //   enableHighAccuracy: true, // Optional: to get more accurate GPS data
                    //   maximumAge: 10000,        // Time in milliseconds before the position is considered outdated
                    //   timeout: 5000             // Timeout if the position cannot be obtained within this time
                    // });
        //     //alert("exit");
        // } else {
        //     alert("Geolocation not supported.");
        // }
      };

      ws.onmessage = function(event) {
        var data = event.data.split(",");

        document.getElementById("lat").innerText = data[0];
        document.getElementById("lon").innerText = data[1];
        //document.getElementById("lat_phone").innerText = data[2] || "N/A";;
        //document.getElementById("lon_phone").innerText = data[3] || "N/A";;
        document.getElementById("RTK_indicator").innerText = data[4] === "1" ? "RTK enabled" : "RTK disabled";
        //document.getElementById("counter").innerText = data[5];
        //document.getElementById("incrementor").innerText = data[6];

        document.getElementById("NTRIP_conn").innerText = data[7] === "1" ? "Connected" : "Not connected";

        let text_fixType = "Blank"
        if (data[8] == 0)
          text_fixType = "None";
        else if (data[8] == 1)
          text_fixType = "Dead Reckoning";
        else if (data[8] == 2)
          text_fixType = "2D";
        else if (data[8] == 3)
          text_fixType = "3D";
        else if (data[8] == 4)
          text_fixType = "GNSS + Dead Reckoning";
        else if (data[8] == 5)
          text_fixType = "Time Only";
        else
          text_fixType = "UNKNOWN";
        document.getElementById("RTK_fix").innerText = text_fixType;

        let text_carrier = "Blank"
        if (data[9] == 0) 
          text_carrier = "None";
        else if (data[9] == 1)
          text_carrier = "Floating";
        else if (data[9] == 2)
          text_carrier = "Fixed";
        else
          text_carrier = "UNKNOWN";
        document.getElementById("RTK_carrier").innerText = text_carrier;

        document.getElementById("horizontal_accuracy").innerText = data[10] + "m";

        document.getElementById("dist").innerText = data[11] + "m";
        document.getElementById("waypoint_button").innerText = data[12] + " " + data[13];
      };
    }

    function sendLocation(position) {
      alert("location");
      alert(position);
      // alert("Your current position is:");
      // alert(`Latitude : ${position.coords.latitude}`);
      // alert(`Longitude: ${position.coords.longitude}`);
      // alert(`More or less ${position.coords.accuracy} meters.`);
      // let data = JSON.stringify({ lat: position.coords.latitude, lon: position.coords.longitude });
      // alert(data);
      // ws.send(data);
      // document.getElementById("status").innerText = "Location Sent: " + data;
    }  
    
    function requestLocation() {
      if (navigator.geolocation) {
        navigator.geolocation.getCurrentPosition(sendLocation, error, { enableHighAccuracy: true });
      } else {
        alert("Geolocation not supported.");
      }
    }
    function error(err) { alert(`ERROR(${err.code}): ${err.message}`);}
    function recordWaypoint() { 
      ws.send("recordWaypoint");
      alert("waypoint recorded");
    }
    function toggle_RTK() { ws.send("toggle"); }
    function increment() { ws.send("increment"); }
    window.onload = init();
  </script>
</body>
</html>
)rawliteral";

// Send updates to all clients
void notifyClients() {
    String message = String(lat_current, 8) + "," + String(lon_current, 8) + "," + 
    String(lat_phone) + "," + String(lon_phone) + "," + 
    (enable_RTK ? "1" : "0") + "," + 
    String(counter) + "," + 
    String(incrementor) + "," +
    String(NTRIP_conn) + "," +
    String(RTK_fix) + "," +
    String(RTK_carrier) + "," +
    String(horizontal_accuracy) + "," +
    String(distance_to_waypoint, 4) + "," +
    String(lat_waypoint, 8) + "," + String(lon_waypoint, 8);
    
    //Serial.println("Sending WebSocket message: " + message);
    ws.textAll(message);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    // grab message from event
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->opcode == WS_TEXT) {
        String message = "";
        for (size_t i = 0; i < len; i++) {
            message += (char)data[i];
        }
//   if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
//     data[len] = 0;
//     if (strcmp((char*)data, "toggle") == 0) {
//       ledState = !ledState;
//       notifyClients();
//     }
//   }
        // check message
        if (message == "toggle") {
            enable_RTK = !enable_RTK;
            Serial.printf("RTK: %s\n", enable_RTK ? "enabled" : "disabled");
        } else if (message == "increment") {
            counter++;
            Serial.printf("Counter: %d\n", counter);
        } else if (message == "recordWaypoint") {
          lat_waypoint = lat_current;
          lon_waypoint = lon_current;          
        }
        // else if (message.startsWith("GPS:")) {
        //   phoneGPS = message.substring(4);  // Extract the GPS data from the message
        //   Serial.println("Received GPS data: " + phoneGPS);
        // } 
        
//         if (receivedMessage == "toggle") {
//             toggleState = !toggleState;
//             ws.textAll(toggleState ? "Toggled: ON" : "Toggled: OFF");
//         } else {
//             message = "Received: " + receivedMessage;
//             ws.textAll(message);
//         }
//     }

        // update clients
        notifyClients();
    }
}
// When an event happens
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.println("WebSocket Client Connected\n\n\n");
        notifyClients();
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.println("WebSocket Client Disconnected");
        //need to kick old clients, timeout?
    } else if (type == WS_EVT_DATA) {
      handleWebSocketMessage(arg, data, len);
  }
}
// void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
//     if (type == WStype_TEXT) {
//         String message = String((char*)payload);
//         if (message == "start") {
//             Serial.println("Start Task Triggered");
//             // Add code to start a task (like enabling GNSS, etc.)
//         } else if (message == "stop") {
//             Serial.println("Stop Task Triggered");
//             // Add code to stop a task
//         } else if (message == "toggleButton") {
//             // Toggle the button state
//             buttonState = !buttonState;
            
//             // Print to serial monitor
//             Serial.print("Button state: ");
//             Serial.println(buttonState ? "ON" : "OFF");

//             // Send updated button state to webpage
//             String jsonData = "{\"buttonState\":\"" + String(buttonState ? "ON" : "OFF") + "\"}";
//             webSocket.broadcastTXT(jsonData);
//         }
//     }
// }

void connectToNTRIP() {
  if (ntripClient.connect(casterHost, casterPort) == false) {
    Serial.println("Connection to caster failed");
  } else {
    Serial.print("Connected to ");
    Serial.print(casterHost);
    Serial.print(" : ");
    Serial.print(casterPort);
    Serial.print(" mount point : ");
    Serial.println(mountPoint);

    // Set up the server request (GET)
    const int SERVER_BUFFER_SIZE = 512;
    char serverRequest[SERVER_BUFFER_SIZE + 1];
    snprintf(serverRequest, SERVER_BUFFER_SIZE, "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP SparkFun u-blox Client v1.0\r\n", mountPoint);

    // Set up the credentials
    char credentials[512];
    if (strlen(casterUser) == 0) {
      strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials));
    } else {
      //Pass base64 encoded user:pw
      char userCredentials[sizeof(casterUser) + sizeof(casterUserPW) + 1]; //The ':' takes up a spot
      snprintf(userCredentials, sizeof(userCredentials), "%s:%s", casterUser, casterUserPW);
      // Serial.print(F("Sending credentials: "));
      // Serial.println(userCredentials);

      //Encode with ESP32 built-in library
      base64 b;
      String strEncodedCredentials = b.encode(userCredentials);
      char encodedCredentials[strEncodedCredentials.length() + 1];
      strEncodedCredentials.toCharArray(encodedCredentials, sizeof(encodedCredentials)); //Convert String to char array
      snprintf(credentials, sizeof(credentials), "Authorization: Basic %s\r\n", encodedCredentials);
    }

    // Add the encoded credentials to the server request
    strncat(serverRequest, credentials, SERVER_BUFFER_SIZE);
    strncat(serverRequest, "\r\n", SERVER_BUFFER_SIZE);
    // Serial.print(F("serverRequest size: "));
    // Serial.print(strlen(serverRequest));
    // Serial.print(F(" of "));
    // Serial.print(sizeof(serverRequest));
    // Serial.println(F(" bytes available"));

    // Send the server request
    // Serial.println(F("Sending server request: "));
    // Serial.println(serverRequest);
    ntripClient.write(serverRequest, strlen(serverRequest));

    //Wait up to 5 seconds for response
    unsigned long startTime = millis();
    while (ntripClient.available() == 0)
    {
      if (millis() > (startTime + 5000))
      {
        Serial.println(F("Caster timed out!"));
        ntripClient.stop();
      }
      delay(10);
    }

    //Check reply
    int connectionResult = 0;
    char response[512];
    size_t responseSpot = 0;
    while (ntripClient.available()) // Read bytes from the caster and store them
    {
      if (responseSpot == sizeof(response) - 1) // Exit the loop if we get too much data
        break;

      response[responseSpot++] = ntripClient.read();

      if (connectionResult == 0) // Only print success/fail once
      {
        if (strstr(response, "200") != nullptr) //Look for '200 OK'
        {
          connectionResult = 200;
        }
        if (strstr(response, "401") != nullptr) //Look for '401 Unauthorized'
        {
          Serial.println(F("Hey - your credentials look bad! Check your caster username and password."));
          connectionResult = 401;
        }
      }
    }
    response[responseSpot] = '\0'; // NULL-terminate the response

    //Serial.print(F("Caster responded with: ")); Serial.println(response); // Uncomment this line to see the full response

    if (connectionResult != 200)
    {
      Serial.print(F("Failed to connect to "));
      Serial.println(casterHost);
      //return;
    }
    else
    {
      Serial.print(F("Connected to: "));
      Serial.println(casterHost);
      lastReceivedRTCM_ms = millis(); //Reset timeout
    }
  }
}

//Check for the arrival of any correction data. Push it to the GNSS.
//Return false if: the connection has dropped, or if we receive no data for maxTimeBeforeHangup_ms
bool processConnection() {
  if (ntripClient.connected() == true) // Check that the connection is still open
  {
    uint8_t rtcmData[512 * 4]; //Most incoming data is around 500 bytes but may be larger
    size_t rtcmCount = 0;

    //Collect any available RTCM data
    while (ntripClient.available())
    {
      //Serial.write(ntripClient.read()); //Pipe to serial port is fine but beware, it's a lot of binary data!
      rtcmData[rtcmCount++] = ntripClient.read();
      if (rtcmCount == sizeof(rtcmData))
        break;
    }

    if (rtcmCount > 0)
    {
      lastReceivedRTCM_ms = millis();

      //Push RTCM to GNSS module over I2C
      myGNSS.pushRawData(rtcmData, rtcmCount);
      
      // Serial.print(F("Pushed "));
      // Serial.print(rtcmCount);
      // Serial.println(F(" RTCM bytes to ZED"));
    }
  }
  else
  {
    Serial.println(F("Connection dropped!"));
     NTRIP_conn = 0;
    return (false); // Connection has dropped - return false
  }  
  
  //Timeout if we don't have new data for maxTimeBeforeHangup_ms
  if ((millis() - lastReceivedRTCM_ms) > maxTimeBeforeHangup_ms)
  {
    Serial.println(F("RTCM timeout!"));
    return (false); // Connection has timed out - return false
  }

  return (true);
}


// Read RTCM data from NTRIP caster and send to ZED-F9R
// void readRTCM() {
//     uint8_t rtcmBuffer[512];  // Buffer for RTCM data
//     size_t bytesRead = 0;
//     while (ntripClient.available()) {
//         rtcmBuffer[bytesRead++] = ntripClient.read();
//         // Send when buffer is full or after a batch of data
//         if (bytesRead >= sizeof(rtcmBuffer)) {
//             // Convert buffer to a String and send as RTCM message
//             String rtcmMessage = "RTCM Data: ";
//             for (size_t i = 0; i < bytesRead; i++) {
//                 rtcmMessage += String(rtcmBuffer[i], HEX) + " ";
//             }
//             sendRTCMMessage(rtcmMessage);  // Send to webpage
//             myGNSS.pushRawData(rtcmBuffer, bytesRead, true);  // Process RTCM data
//             bytesRead = 0;
//         }
//     }
//     // Send remaining bytes if any
//     if (bytesRead > 0) {
//         String rtcmMessage = "RTCM Data: ";
//         for (size_t i = 0; i < bytesRead; i++) {
//             rtcmMessage += String(rtcmBuffer[i], HEX) + " ";
//         }
//         sendRTCMMessage(rtcmMessage);  // Send to webpage
//         myGNSS.pushRawData(rtcmBuffer, bytesRead, true);
//     }
// }

void setup() {
  Serial.begin(115200);
  Wire.begin();
  while (!myGNSS.begin()) {
    Serial.println("Connecting to GNSS...");
    delay(2000);
  }

  WiFi.begin(ssid, password);
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send_P(200, "text/html", myWebpage);
  });
  server.begin();

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA);                                //Set the I2C port to output both NMEA and UBX messages
  myGNSS.setI2CInput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); //Be sure RTCM3 input is enabled. UBX + RTCM3 is not a valid state.
  myGNSS.setDGNSSConfiguration(SFE_UBLOX_DGNSS_MODE_FIXED); // Set the differential mode - ambiguities are fixed whenever possible
  myGNSS.setNavigationFrequency(20); //Set output in Hz.
    // Set the Main Talker ID to "GP". The NMEA GGA messages will be GPGGA instead of GNGGA
  myGNSS.setMainTalkerID(SFE_UBLOX_MAIN_TALKER_ID_GP);
  myGNSS.setNMEAGPGGAcallbackPtr(&pushGPGGA); // Set up the callback for GPGGA
  myGNSS.setVal8(UBLOX_CFG_MSGOUT_NMEA_ID_GGA_I2C, 20); // Tell the module to output GGA every 10 seconds
  myGNSS.setAutoPVTcallbackPtr(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata so we can watch the carrier solution go to fixed
  myGNSS.setRTCM1005InputcallbackPtr(&printRTCMdata1005); // Set up a callback to print the RTCM 1005 Antenna Reference Position from the correction data
  myGNSS.setRTCM1006InputcallbackPtr(&printRTCMdata1006); // Set up a callback to print the RTCM 1006 Antenna Reference Position from the correction data

}

void loop() {
  // Handle wifi connection
  while (WiFi.status() != WL_CONNECTED) {
    // WiFi.begin(ssid, password); take new ssid & pass for other users
    Serial.println("Stop car... Connecting to WiFi...");

    delay(500);
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("Connected to %s\nESP32 IP Address: ", ssid);
      Serial.println(WiFi.localIP());
    }
  }
  while (enable_RTK && !ntripClient.connected()) {
    Serial.println(F("Connecting to the NTRIP caster..."));
    connectToNTRIP();
    NTRIP_conn = 1;
  }
  if (!enable_RTK && ntripClient.connected()) {
    ntripClient.stop();
    NTRIP_conn = 0;
  }



  myGNSS.checkUblox(); // Check for the arrival of new GNSS data and process it.
  myGNSS.checkCallbacks(); // Check if any GNSS callbacks are waiting to be processed.
  processConnection();

  distance_to_waypoint = calculateDistance(lat_current, lon_current, lat_waypoint, lon_waypoint);

  //Serial.printf(":  lat: %.8f lon: %.8f", lat_current, lon_current);
  incrementor++;

  notifyClients();
  delay(1000);
  //ws.cleanupClients();  // Clean up inactive WebSocket clients     do when swiching hotspot?
}
