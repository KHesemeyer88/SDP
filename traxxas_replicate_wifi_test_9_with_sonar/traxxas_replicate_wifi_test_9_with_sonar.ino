#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>

// Pin Definitions
const int STEERING_PIN = 5;   // GPIO5 for steering servo
const int ESC_PIN = 23;      // GPIO23 for ESC control
// Sonar pins
const int TRIGGER_PIN_FRONT = 13;
const int ECHO_PIN_FRONT = 14;
const int TRIGGER_PIN_LEFT = 25;
const int ECHO_PIN_LEFT = 26;
const int TRIGGER_PIN_RIGHT = 27;
const int ECHO_PIN_RIGHT = 12;

// Sonar thresholds (cm)
const int FRONT_STOP_THRESHOLD = 50;
const int SIDE_AVOID_THRESHOLD = 50;

// GPS setup
float targetLat = 0.0;
float targetLon = 0.0;
bool autonomousMode = false;
SFE_UBLOX_GNSS myGPS;

// Servo objects
Servo steeringServo;
Servo escServo;

// Last sonar readings
float lastFrontDist = 0;
float lastLeftDist = 0;
float lastRightDist = 0;
String lastAvoidanceMessage = "";

// Sonar filtering
const int FILTER_SAMPLES = 5;  // Number of samples to average
float frontReadings[FILTER_SAMPLES] = {0};
float leftReadings[FILTER_SAMPLES] = {0};
float rightReadings[FILTER_SAMPLES] = {0};
int readIndex = 0;
unsigned long lastAvoidanceTime = 0;
const unsigned long AVOIDANCE_MESSAGE_TIMEOUT = 1000; // Clear message after 1 second
bool inLeftAvoidance = false;
bool inRightAvoidance = false;
unsigned long avoidanceStartTime = 0;
const unsigned long AVOIDANCE_DURATION = 1000; // Continue avoiding for 1 second

// WiFi settings
const char* ssid = "RC_Car_Control";
const char* password = "12345678";

WebServer server(80);

// Timing variables
unsigned long lastUpdateTime = 0;
const unsigned long TIMEOUT_MS = 200;

// Traxxas XL-2.5 ESC values
const int ESC_NEUTRAL = 90;     // Neutral position (1.5ms pulse)
const int ESC_MAX_FWD = 130;    // Full forward (~1.9ms pulse)
const int ESC_MAX_REV = 50;     // Full reverse (~1.1ms pulse)
const int ESC_MIN_FWD = 95;     // Minimum forward throttle
const int ESC_MIN_REV = 85;     // Minimum reverse throttle
const int STEERING_CENTER = 90;  // Center steering
const int STEERING_MAX = 45;     // Maximum steering angle deviation

// Function declarations (prototypes)
float calculateDistance(float lat1, float lon1, float lat2, float lon2);
float calculateBearing(float lat1, float lon1, float lat2, float lon2);

// HTML/js for webpage
const char webPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>RC Car Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
            display: flex;
            flex-direction: column;
            align-items: center;
            padding: 20px;
            font-size: 24px;
            font-family: Arial, sans-serif;
            height: 100vh;
            margin: 0;
            background-color: #f4f4f9;
        }
        .control-container {
            width: 100%;
            max-width: 800px;
            display: flex;
            flex-direction: column;
            align-items: center;
        }
        .mode-switch {
            margin: 20px 0;
            padding: 10px;
            background: white;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        .mode-switch button {
            padding: 10px 20px;
            font-size: 18px;
            margin: 0 10px;
            border: none;
            border-radius: 4px;
            cursor: pointer;
        }
        .active {
            background-color: #007bff;
            color: white;
        }
        .inactive {
            background-color: #e9ecef;
            color: #495057;
        }
        canvas {
            background: #ddd;
            border: 2px solid #444;
            border-radius: 4px;
        }
        #gps-data, #autonomous-control {
            background: white;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            margin-top: 20px;
            width: 80%;
            text-align: center;
        }
        #autonomous-control {
            display: none;
        }
        #manual-control {
            display: flex;
            flex-direction: column;
            align-items: center;
        }
        .coordinate-input {
            margin: 10px 0;
            padding: 10px;
            font-size: 18px;
            width: 80%;
            max-width: 300px;
            border: 1px solid #ddd;
            border-radius: 4px;
        }
        .submit-btn {
            padding: 10px 20px;
            font-size: 18px;
            background-color: #28a745;
            color: white;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            margin-top: 10px;
        }
        .submit-btn:hover {
            background-color: #218838;
        }
    </style>
</head>
<body>
    <div class="control-container">
        <div class="mode-switch">
            <button id="manual-btn" class="active" onclick="switchMode('manual')">Manual Control</button>
            <button id="auto-btn" class="inactive" onclick="switchMode('autonomous')">Autonomous Mode</button>
        </div>

        <div id="manual-control">
            <div id="sensor-readout" style="font-size: 14px; margin-bottom: 10px; font-family: monospace;">
                Front: -- cm | Left: -- cm | Right: -- cm
            </div>
            <canvas id="joystick" width="300" height="300"></canvas>
        </div>

        <div id="autonomous-control">
            <h2>Set Destination</h2>
            <input type="text" id="coords-input" class="coordinate-input" placeholder="Coordinates (e.g. 42.637088, -72.729328)">
            <button class="submit-btn" onclick="startAutonomousMode()">Start Navigation</button>
            <button class="submit-btn" style="background-color: #dc3545;" onclick="stopAutonomousMode()">Stop Navigation</button>
            <div id="avoidance-alert" style="display: none; margin-top: 10px; padding: 10px; background-color: #ffc107; border-radius: 4px;"></div>
        </div>

        <div id="gps-data">
            <p>Distance to Target: <span id="distance">--</span> m</p>
            <p>Current Bearing: <span id="bearing">--</span>&deg;</p>
            <p>Fix Status: <span id="fix">No Fix</span></p>
            <p style="margin-top: 20px;">Target Location:</p>
            <p>Latitude: <span id="dest-lat">--</span></p>
            <p>Longitude: <span id="dest-lng">--</span></p>
        </div>
    </div>

<script>
        const canvas = document.getElementById('joystick');
        const ctx = canvas.getContext('2d');
        const joystickSize = 150;
        const handleRadius = 30;
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;

        let handleX = centerX;
        let handleY = centerY;
        let isDragging = false;
        let updateInterval = null;

        function drawJoystick() {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.strokeStyle = '#bbb';
            ctx.lineWidth = 4;
            ctx.strokeRect(centerX - joystickSize, centerY - joystickSize, joystickSize * 2, joystickSize * 2);
            ctx.beginPath();
            ctx.arc(handleX, handleY, handleRadius, 0, Math.PI * 2);
            ctx.fillStyle = '#444';
            ctx.fill();
        }

        function switchMode(mode) {
            if (mode === 'manual') {
                document.getElementById('manual-control').style.display = 'flex';
                document.getElementById('autonomous-control').style.display = 'none';
                document.getElementById('manual-btn').className = 'active';
                document.getElementById('auto-btn').className = 'inactive';
                stopAutonomousMode();
            } else {
                document.getElementById('manual-control').style.display = 'none';
                document.getElementById('autonomous-control').style.display = 'block';
                document.getElementById('manual-btn').className = 'inactive';
                document.getElementById('auto-btn').className = 'active';
            }
        }

        function startAutonomousMode() {
            const coordsStr = document.getElementById('coords-input').value.trim();
            const coords = coordsStr.split(',').map(coord => coord.trim());
            
            if (coords.length !== 2 || isNaN(coords[0]) || isNaN(coords[1])) {
                alert('Please enter valid coordinates');
                return;
            }
            
            const lat = parseFloat(coords[0]);
            const lng = parseFloat(coords[1]);

            fetch(`/setDestination?lat=${lat}&lng=${lng}`)
                .then(response => response.text())
                .then(result => {
                    console.log('Navigation started:', result);
                })
                .catch(error => {
                    console.error('Error starting navigation:', error);
                    alert('Failed to start navigation');
                });
        }

        function stopAutonomousMode() {
            fetch('/stopNavigation')
                .then(response => response.text())
                .then(result => {
                    console.log('Navigation stopped:', result);
                })
                .catch(error => {
                    console.error('Error stopping navigation:', error);
                });
        }

        // Add both mouse and touch events
        canvas.addEventListener('mousedown', (e) => {
            isDragging = true;
            if (updateInterval === null) {
                updateInterval = setInterval(sendUpdate, 100);
            }
            handleMouseMove(e);
        });

        canvas.addEventListener('mousemove', (e) => {
            if (isDragging) {
                handleMouseMove(e);
            }
        });

        canvas.addEventListener('mouseup', () => {
            handleEnd();
        });

        canvas.addEventListener('touchstart', (e) => {
            e.preventDefault();
            isDragging = true;
            if (updateInterval === null) {
                updateInterval = setInterval(sendUpdate, 100);
            }
            handleTouchMove(e);
        });

        canvas.addEventListener('touchmove', (e) => {
            e.preventDefault();
            if (isDragging) {
                handleTouchMove(e);
            }
        });

        canvas.addEventListener('touchend', () => {
            handleEnd();
        });

        function handleMouseMove(e) {
            const rect = canvas.getBoundingClientRect();
            const mouseX = e.clientX - rect.left;
            const mouseY = e.clientY - rect.top;
            updateJoystickPosition(mouseX, mouseY);
        }

        function handleTouchMove(e) {
            const touch = e.touches[0];
            const rect = canvas.getBoundingClientRect();
            const touchX = touch.clientX - rect.left;
            const touchY = touch.clientY - rect.top;
            updateJoystickPosition(touchX, touchY);
        }

        function handleEnd() {
            isDragging = false;
            if (updateInterval !== null) {
                clearInterval(updateInterval);
                updateInterval = null;
            }
            handleX = centerX;
            handleY = centerY;
            drawJoystick();
            sendUpdate();
        }

        function updateJoystickPosition(x, y) {
            const dx = Math.min(Math.max(x - centerX, -joystickSize), joystickSize);
            const dy = Math.min(Math.max(y - centerY, -joystickSize), joystickSize);
            handleX = centerX + dx;
            handleY = centerY + dy;
            drawJoystick();
        }

        function calculateValues() {
            const dx = handleX - centerX;
            const dy = handleY - centerY;
            const maxDistance = joystickSize;

            const speed = Math.round((-dy / maxDistance) * 255);
            const angle = Math.round((dx / maxDistance) * 45 + 90);

            return { speed, angle };
        }

        function sendUpdate() {
            const { speed, angle } = calculateValues();
            fetch(`/control?speed=${speed}&angle=${angle}`)
                .catch((e) => console.error('Error:', e));
        }

        drawJoystick();

        // GPS status updates
        setInterval(() => {
            fetch('/gps')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('fix').textContent = data.fix;
                    document.getElementById('distance').textContent = data.distance;
                    document.getElementById('bearing').textContent = data.bearing;
                    document.getElementById('dest-lat').textContent = data.destLat || '--';
                    document.getElementById('dest-lng').textContent = data.destLng || '--';
                })
                .catch(console.error);
        }, 1000);

        // Update sensor readings
        setInterval(() => {
            fetch('/sensors')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('sensor-readout').textContent = 
                        `Front: ${data.front} cm | Left: ${data.left} cm | Right: ${data.right} cm`;
                    
                    if (data.message && data.message !== "") {
                        const alert = document.getElementById('avoidance-alert');
                        alert.textContent = data.message;
                        alert.style.display = 'block';
                        setTimeout(() => {
                            alert.style.display = 'none';
                        }, 3000);  // Hide after 3 seconds
                    }
                })
                .catch(console.error);
        }, 200);  // Update every 200ms
    </script>
</body>
</html>
)rawliteral";

// Sonar functions
float readSonar(int trigPin, int echoPin, float* readings) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH, 30000);
    float distance = (duration == 0) ? -1 : duration * 0.034 / 2;
    
    // Only update array if we got a valid reading
    if (distance > 0) {
        readings[readIndex] = distance;
    }
    
    // Calculate median
    float sortedReadings[FILTER_SAMPLES];
    memcpy(sortedReadings, readings, sizeof(sortedReadings));
    for(int i = 0; i < FILTER_SAMPLES-1; i++) {
        for(int j = i + 1; j < FILTER_SAMPLES; j++) {
            if(sortedReadings[j] < sortedReadings[i]) {
                float temp = sortedReadings[i];
                sortedReadings[i] = sortedReadings[j];
                sortedReadings[j] = temp;
            }
        }
    }
    
    return sortedReadings[FILTER_SAMPLES/2];  // Return median value
}

unsigned long lastSonarUpdate = 0;
const unsigned long SONAR_UPDATE_INTERVAL = 100; // 100ms between full sonar updates

void updateSonarReadings() {
    if (millis() - lastSonarUpdate >= SONAR_UPDATE_INTERVAL) {
        lastFrontDist = readSonar(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, frontReadings);
        lastLeftDist = readSonar(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT, leftReadings);
        lastRightDist = readSonar(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, rightReadings);
        
        readIndex = (readIndex + 1) % FILTER_SAMPLES;
        lastSonarUpdate = millis();
    }
}

void setup() {
    Serial.begin(115200);
    //Serial.println("Starting RC Car Control with Sonar...");
    
    // Configure sonar pins
    pinMode(TRIGGER_PIN_FRONT, OUTPUT);
    pinMode(ECHO_PIN_FRONT, INPUT);
    pinMode(TRIGGER_PIN_LEFT, OUTPUT);
    pinMode(ECHO_PIN_LEFT, INPUT);
    pinMode(TRIGGER_PIN_RIGHT, OUTPUT);
    pinMode(ECHO_PIN_RIGHT, INPUT);
    
    // Initialize Servo library
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    
    // Configure servo objects with explicit timing
    steeringServo.setPeriodHertz(50);
    escServo.setPeriodHertz(50);
    
    steeringServo.attach(STEERING_PIN, 1000, 2000);
    escServo.attach(ESC_PIN, 1000, 2000);
    
    // Initialize ESC - start in neutral
    //Serial.println("Setting ESC to neutral");
    escServo.write(ESC_NEUTRAL);
    delay(3000);  // Give ESC time to initialize
    
    // Center steering
    steeringServo.write(STEERING_CENTER);
    
    // Initialize GPS
    Wire.begin(32, 33);
    if (myGPS.begin() == false) {
        //Serial.println("u-blox GNSS not detected. Check wiring.");
    }

    // Configure GPS
    myGPS.setI2COutput(COM_TYPE_UBX);
    myGPS.setNavigationFrequency(10);
    myGPS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
    //Serial.println("GPS initialization complete");

    // Setup WiFi Access Point
    WiFi.softAP(ssid, password);
    Serial.println("Access Point Started");
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());

    // Web server routes
    server.on("/", HTTP_GET, []() {
        Serial.println("Root page requested");
        server.send(200, "text/html", webPage);
    });

    server.on("/control", HTTP_GET, []() {
        String speedStr = server.arg("speed");
        String angleStr = server.arg("angle");

        if (speedStr != "" && angleStr != "") {
            int speed = speedStr.toInt();
            int angle = angleStr.toInt();
            
            // Convert -255 to 255 speed range to ESC range
            int escValue;
            if (abs(speed) < 20) {  // Small deadzone for neutral
                escValue = ESC_NEUTRAL;
            } else if (speed > 0) {
                escValue = map(speed, 20, 255, ESC_MIN_FWD, ESC_MAX_FWD);
            } else {
                escValue = map(speed, -255, -20, ESC_MAX_REV, ESC_MIN_REV);
            }
            
            escServo.write(escValue);
            steeringServo.write(angle);
            
            lastUpdateTime = millis();
            server.send(200, "text/plain", "OK");
        } else {
            server.send(400, "text/plain", "Missing parameters");
        }
    });

    // endpoint for sensor data
    server.on("/sensors", HTTP_GET, []() {
        String json = "{\"front\":" + String(lastFrontDist, 1) +
                     ",\"left\":" + String(lastLeftDist, 1) +
                     ",\"right\":" + String(lastRightDist, 1) +
                     ",\"message\":\"" + lastAvoidanceMessage + "\"}";
        if (lastAvoidanceMessage != "Destination reached") {
            lastAvoidanceMessage = "";
        }
        server.send(200, "application/json", json);
    });

    server.on("/gps", HTTP_GET, []() {
        float currentLat = myGPS.getLatitude() / 10000000.0;
        float currentLon = myGPS.getLongitude() / 10000000.0;
        
        float distance = 0;
        float bearing = 0;
        bool isValid = (myGPS.getFixType() > 0);
        
        if (isValid && (targetLat != 0 || targetLon != 0)) {
            distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
            bearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
        }

        String json = "{\"distance\":" + String(distance, 1) +
                      ",\"bearing\":" + String(bearing, 1) +
                      ",\"valid\":" + String(isValid) +
                      ",\"fix\":\"" + String(isValid ? "Active" : "No Fix") + "\"" +
                      ",\"destLat\":" + String(targetLat, 6) +
                      ",\"destLng\":" + String(targetLon, 6) +
                      "}";
        server.send(200, "application/json", json);
    });

    server.on("/setDestination", HTTP_GET, []() {
        String lat = server.arg("lat");
        String lng = server.arg("lng");
        
        targetLat = lat.toFloat();
        targetLon = lng.toFloat();
        autonomousMode = true;
        lastAvoidanceMessage = "";  // Clear any previous messages
        
        server.send(200, "text/plain", "Navigation started");
    });

    server.on("/stopNavigation", HTTP_GET, []() {
        autonomousMode = false;
        escServo.write(ESC_NEUTRAL);
        steeringServo.write(STEERING_CENTER);
        server.send(200, "text/plain", "Navigation stopped");
    });

    server.begin();

}

void loop() {
    // Update sonar readings
    updateSonarReadings();
    
    // Check GPS
    myGPS.checkUblox();
    myGPS.checkCallbacks();

    // Autonomous navigation with obstacle avoidance
    if (autonomousMode && myGPS.getFixType() > 0) {
        float currentLat = myGPS.getLatitude() / 10000000.0;
        float currentLon = myGPS.getLongitude() / 10000000.0;
        
        float distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
        float bearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
        float currentHeading = myGPS.getHeading() / 100000.0;
        
        float headingError = bearing - currentHeading;
        if (headingError > 180) headingError -= 360;
        if (headingError < -180) headingError += 360;
        
        // Base steering calculation
        int steeringAngle = STEERING_CENTER + (headingError * 0.25);
        
        // Modify steering for obstacle avoidance
        if (lastFrontDist > 0 && lastFrontDist < FRONT_STOP_THRESHOLD) {
            // Front obstacle detected - stop
            escServo.write(ESC_NEUTRAL);
            lastAvoidanceMessage = "Stopped - Front obstacle detected";
            lastAvoidanceTime = millis();
            //Serial.println("Front obstacle detected - stopping");
        } else {
            // Check side obstacles and modify steering if needed
            if (lastLeftDist > 0 && lastLeftDist < SIDE_AVOID_THRESHOLD) {
                inLeftAvoidance = true;
                inRightAvoidance = false;  // Cancel any right avoidance
                avoidanceStartTime = millis();
                lastAvoidanceMessage = "Avoiding left obstacle";
                lastAvoidanceTime = millis();
                //Serial.println("Left obstacle detected - steering right");
            }
            if (lastRightDist > 0 && lastRightDist < SIDE_AVOID_THRESHOLD) {
                // Obstacle on right - steer left
                inRightAvoidance = true;
                inLeftAvoidance = false;  // Cancel any left avoidance
                avoidanceStartTime = millis();
                lastAvoidanceMessage = "Avoiding right obstacle";
                lastAvoidanceTime = millis();
                //Serial.println("Right obstacle detected - steering left");
            }
            // Apply steering changes based on avoidance state
            if (inLeftAvoidance && (millis() - avoidanceStartTime < AVOIDANCE_DURATION)) {
                steeringAngle += 15;  // Steer away from left obstacle
            } else {
                inLeftAvoidance = false;
            }

            if (inRightAvoidance && (millis() - avoidanceStartTime < AVOIDANCE_DURATION)) {
                steeringAngle -= 15;  // Steer away from right obstacle
            } else {
                inRightAvoidance = false;
            }
            // Apply normal speed if no front obstacle
            int speed = 128;  // Base speed
            if (distance < 1.0) {
                speed = 0;
                autonomousMode = false;
                lastAvoidanceMessage = "Destination reached";
            }
            
            // Convert to ESC value and apply
            int escValue = map(speed, 0, 255, ESC_MIN_FWD, ESC_MAX_FWD);
            escServo.write(escValue);
        }
        
        // Apply final steering angle with constraints
        steeringAngle = constrain(steeringAngle, 
                                STEERING_CENTER - STEERING_MAX, 
                                STEERING_CENTER + STEERING_MAX);
        steeringServo.write(steeringAngle);
        lastUpdateTime = millis();
    }
    // Clear stale avoidance messages
    if (lastAvoidanceMessage != "" && millis() - lastAvoidanceTime > AVOIDANCE_MESSAGE_TIMEOUT) {
        lastAvoidanceMessage = "";
    }
    server.handleClient();
    
    // Safety timeout
    if (millis() - lastUpdateTime > TIMEOUT_MS) {
        escServo.write(ESC_NEUTRAL);
        steeringServo.write(STEERING_CENTER);
    }

    delay(2);
}

// Haversine distance calculation
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

float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    float lat1Rad = lat1 * PI / 180.0;
    float lon1Rad = lon1 * PI / 180.0;
    float lat2Rad = lat2 * PI / 180.0;
    float lon2Rad = lon2 * PI / 180.0;

    float dLon = lon2Rad - lon1Rad;
    float y = sin(dLon) * cos(lat2Rad);
    float x = cos(lat1Rad) * sin(lat2Rad) -
              sin(lat1Rad) * cos(lat2Rad) * cos(dLon);
    
    float bearing = atan2(y, x) * 180.0 / PI;
    return fmod((bearing + 360.0), 360.0);
}

