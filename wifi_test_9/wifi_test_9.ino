#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <SparkFun_u-blox_GNSS_v3.h>

//GPS
float targetLat = 0.0;
float targetLon = 0.0;
bool autonomousMode = false;  // Flag to indicate if we're in autonomous mode
SFE_UBLOX_GNSS myGPS;

// Motor A (Front) control pins
const int MOTOR_A_PWM = 23;
const int MOTOR_A_IN1 = 21;
const int MOTOR_A_IN2 = 22;
// Motor B (Rear) control pins
const int MOTOR_B_PWM = 5;
const int MOTOR_B_IN1 = 19;
const int MOTOR_B_IN2 = 18;
// Servo setup
const int SERVO_PIN = 4;
Servo steering;

unsigned long lastUpdateTime = 0;  // Tracks the last time an update was received
const unsigned long TIMEOUT_MS = 200;  // Timeout period in milliseconds

// WiFi settings
const char* ssid = "RC_Car_Control";
const char* password = "12345678";

WebServer server(80);

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
            <canvas id="joystick" width="300" height="300"></canvas>
        </div>

        <div id="autonomous-control">
            <h2>Set Destination</h2>
            <input type="text" id="coords-input" class="coordinate-input" placeholder="Coordinates (e.g. 42.637088, -72.729328)">
            <button class="submit-btn" onclick="startAutonomousMode()">Start Navigation</button>
            <button class="submit-btn" style="background-color: #dc3545;" onclick="stopAutonomousMode()">Stop Navigation</button>
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

        canvas.addEventListener('touchstart', (e) => {
            isDragging = true;
            if (updateInterval === null) {
                updateInterval = setInterval(sendUpdate, 100);
            }
        });

        canvas.addEventListener('touchmove', (e) => {
            e.preventDefault();
            if (!isDragging) return;

            const touch = e.touches[0];
            const rect = canvas.getBoundingClientRect();
            const touchX = touch.clientX - rect.left;
            const touchY = touch.clientY - rect.top;

            const dx = Math.min(Math.max(touchX - centerX, -joystickSize), joystickSize);
            const dy = Math.min(Math.max(touchY - centerY, -joystickSize), joystickSize);

            handleX = centerX + dx;
            handleY = centerY + dy;

            drawJoystick();
        });

        canvas.addEventListener('touchend', () => {
            isDragging = false;
            if (updateInterval !== null) {
                clearInterval(updateInterval);
                updateInterval = null;
            }
            handleX = centerX;
            handleY = centerY;
            drawJoystick();
            sendUpdate();
        });

        function calculateValues() {
            const dx = handleX - centerX;
            const dy = handleY - centerY;
            const maxDistance = joystickSize;

            const speed = Math.round((-dy / maxDistance) * 255);
            const angle = Math.round((dx / maxDistance) * 35 + 90);

            return { speed, angle };
        }

        function sendUpdate() {
            const { speed, angle } = calculateValues();
            fetch(`/control?speed=${speed}&angle=${angle}`)
                .catch((e) => console.error(e));
        }

        drawJoystick();

        setInterval(() => {
            fetch('/gps')
                .then(response => response.json())
                .then(data => {
                    console.log('GPS Data received:', data);  // Debug line
                    document.getElementById('fix').textContent = data.fix;
                    document.getElementById('distance').textContent = data.distance;
                    document.getElementById('bearing').textContent = data.bearing;
                    document.getElementById('dest-lat').textContent = data.destLat || '--';
                    document.getElementById('dest-lng').textContent = data.destLng || '--';
                })
                .catch(console.error);
        }, 1000);
    </script>
</body>
</html>
)rawliteral";

void setup() {
    Serial.begin(115200);
    
    // Configure motor pins
    pinMode(MOTOR_A_IN1, OUTPUT);
    pinMode(MOTOR_A_IN2, OUTPUT);
    pinMode(MOTOR_B_IN1, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);
    
    // Setup PWM
    ledcAttach(MOTOR_A_PWM, 1000, 8);
    ledcAttach(MOTOR_B_PWM, 1000, 8);
    
    // Setup servo
    ESP32PWM::allocateTimer(3);
    steering.attach(SERVO_PIN);
    steering.write(90);

    //GPS
    Wire.begin(32,33);
    if (myGPS.begin() == false) {
        // Minimal debug
        Serial.println("u-blox GNSS not detected at default I2C address. Check wiring.");
        // while (1); // Freeze if you want
    }

    // Configure the F9R
    myGPS.setI2COutput(COM_TYPE_UBX);
    myGPS.setNavigationFrequency(10);
    myGPS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
    Serial.println("GPS initialization successful");

    // Setup WiFi Access Point
    WiFi.softAP(ssid, password);
    Serial.println("Access Point Started");
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());

    // Setup web server routes
    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", webPage);
    });

    server.on("/control", HTTP_GET, []() {
        String speed = server.arg("speed");
        String angle = server.arg("angle");

        if (speed != "" && angle != "") {
            setMotorSpeed(speed.toInt(), speed.toInt());
            setSteeringAngle(angle.toInt());
            lastUpdateTime = millis();  // Reset the timeout timer
            server.send(200, "text/plain", "OK");
        } else {
            server.send(400, "text/plain", "Missing parameters");
        }
    });

    server.on("/heartbeat", HTTP_GET, []() {
        lastUpdateTime = millis();  // Reset the timeout timer
        server.send(200, "text/plain", "Heartbeat received");
    });

    server.begin();
    Serial.println("HTTP server started");

    server.on("/gps", HTTP_GET, []() {
        float currentLat = myGPS.getLatitude() / 10000000.0;
        float currentLon = myGPS.getLongitude() / 10000000.0;
        
        // Calculate distance and bearing if fix is valid and we have a target
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
        
        server.send(200, "text/plain", "Navigation started");
    });

    server.on("/stopNavigation", HTTP_GET, []() {
        autonomousMode = false;
        setMotorSpeed(0, 0);
        setSteeringAngle(90);
        server.send(200, "text/plain", "Navigation stopped");
    });
}

void setMotorSpeed(int motorA_speed, int motorB_speed) {
    if (motorA_speed > 0) {
        digitalWrite(MOTOR_A_IN1, LOW);
        digitalWrite(MOTOR_A_IN2, HIGH);
    } else if (motorA_speed < 0) {
        digitalWrite(MOTOR_A_IN1, HIGH);
        digitalWrite(MOTOR_A_IN2, LOW);
    } else {
        digitalWrite(MOTOR_A_IN1, LOW);
        digitalWrite(MOTOR_A_IN2, LOW);
    }

    if (motorB_speed > 0) {
        digitalWrite(MOTOR_B_IN1, HIGH);
        digitalWrite(MOTOR_B_IN2, LOW);
    } else if (motorB_speed < 0) {
        digitalWrite(MOTOR_B_IN1, LOW);
        digitalWrite(MOTOR_B_IN2, HIGH);
    } else {
        digitalWrite(MOTOR_B_IN1, LOW);
        digitalWrite(MOTOR_B_IN2, LOW);
    }

    ledcWrite(MOTOR_A_PWM, abs(motorA_speed));
    ledcWrite(MOTOR_B_PWM, abs(motorB_speed));
}

void setSteeringAngle(int angle) {
    steering.write(angle);
}

// Haversine-based distance
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
    float r = 6371000; // Earth radius in meters
    return c * r;
}

// Bearing
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    float lat1Rad = lat1 * PI / 180.0;
    float lon1Rad = lon1 * PI / 180.0;
    float lat2Rad = lat2 * PI / 180.0;
    float lon2Rad = lon2 * PI / 180.0;

    float dLon = lon2Rad - lon1Rad;
    float y = sin(dLon) * cos(lat2Rad);
    float x = cos(lat1Rad) * sin(lat2Rad)
            - sin(lat1Rad) * cos(lat2Rad) * cos(dLon);
    
    float bearing = atan2(y, x) * 180.0 / PI;
    bearing = fmod((bearing + 360.0), 360.0);
    return bearing;
}

void loop() {
    // Minimal GPS check
    myGPS.checkUblox();
    myGPS.checkCallbacks();

    // If we're in autonomous mode and have a GPS fix
    if (autonomousMode && myGPS.getFixType() > 0) {
        float currentLat = myGPS.getLatitude() / 10000000.0;
        float currentLon = myGPS.getLongitude() / 10000000.0;
        
        float distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
        float bearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
        float currentHeading = myGPS.getHeading() / 100000.0;
        
        float headingError = bearing - currentHeading;
        if (headingError > 180) headingError -= 360;
        if (headingError < -180) headingError += 360;
        
        int steeringAngle = 90 + (headingError * 0.25);
        steeringAngle = constrain(steeringAngle, 55, 125);
        
        int speed = 128;  // Base speed
        if (distance < 1.0) {
            speed = 0;  // Stop at destination
            autonomousMode = false;
        }
        
        setSteeringAngle(steeringAngle);
        setMotorSpeed(speed, speed);
        lastUpdateTime = millis();
        // No debug prints here
    }

    server.handleClient();

    // Add a small delay so WiFi doesn't get starved
    delay(2);

    // Check if the timeout period has elapsed
    if (millis() - lastUpdateTime > TIMEOUT_MS) {
        setMotorSpeed(0, 0);
        setSteeringAngle(90);
    }
}
