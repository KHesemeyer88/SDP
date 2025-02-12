#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>

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

// Ultrasonic sensor pins
const int TRIG_PIN_LEFT = 13;
const int ECHO_PIN_LEFT = 12;
const int TRIG_PIN_CENTER = 14;
const int ECHO_PIN_CENTER = 27;
const int TRIG_PIN_RIGHT = 26;
const int ECHO_PIN_RIGHT = 25;

// GPS
float targetLat = 0.0;
float targetLon = 0.0;
bool autonomousMode = false;  // Flag to indicate if we're in autonomous mode
TinyGPSPlus gps;

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
            <input type="text" id="lat-input" class="coordinate-input" placeholder="Latitude (e.g. 37.7749)">
            <input type="text" id="lng-input" class="coordinate-input" placeholder="Longitude (e.g. -122.4194)">
            <button class="submit-btn" onclick="startAutonomousMode()">Start Navigation</button>
            <button class="submit-btn" style="background-color: #dc3545;" onclick="stopAutonomousMode()">Stop Navigation</button>
        </div>

        <div id="gps-data">
            <p>Current Position:</p>
            <p>Latitude: <span id="lat">--</span></p>
            <p>Longitude: <span id="lng">--</span></p>
            <p>Satellites: <span id="sats">--</span></p>
            <p>Fix Status: <span id="fix">No Fix</span></p>
            <p style="margin-top: 20px;">Destination:</p>
            <p>Latitude: <span id="dest-lat">--</span></p>
            <p>Longitude: <span id="dest-lng">--</span></p>
        </div>
    </div>

    <script>
        // Existing joystick code remains the same
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

        // Mode switching functionality
        function switchMode(mode) {
            if (mode === 'manual') {
                document.getElementById('manual-control').style.display = 'flex';
                document.getElementById('autonomous-control').style.display = 'none';
                document.getElementById('manual-btn').className = 'active';
                document.getElementById('auto-btn').className = 'inactive';
                // Stop autonomous navigation if it's running
                stopAutonomousMode();
            } else {
                document.getElementById('manual-control').style.display = 'none';
                document.getElementById('autonomous-control').style.display = 'block';
                document.getElementById('manual-btn').className = 'inactive';
                document.getElementById('auto-btn').className = 'active';
            }
        }

        function startAutonomousMode() {
            const lat = document.getElementById('lat-input').value;
            const lng = document.getElementById('lng-input').value;
            
            if (!lat || !lng) {
                alert('Please enter both latitude and longitude');
                return;
            }

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

        // Existing joystick event listeners and functionality
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

        // Initialize joystick and start GPS updates
        drawJoystick();
        setInterval(() => {
            fetch('/gps')
                .then(response => response.json())
                .then(data => {
                    if (data.valid) {
                        document.getElementById('lat').textContent = data.lat;
                        document.getElementById('lng').textContent = data.lng;
                    }
                    document.getElementById('sats').textContent = data.sats;
                    document.getElementById('fix').textContent = data.fix;
                    
                    // Update destination coordinates
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

    // Initialize ultrasonic sensor pins
    pinMode(TRIG_PIN_LEFT, OUTPUT);
    pinMode(ECHO_PIN_LEFT, INPUT);
    pinMode(TRIG_PIN_CENTER, OUTPUT);
    pinMode(ECHO_PIN_CENTER, INPUT);
    pinMode(TRIG_PIN_RIGHT, OUTPUT);
    pinMode(ECHO_PIN_RIGHT, INPUT);

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

    server.on("/gps", HTTP_GET, []() {
        String json = "{\"lat\":" + String(gps.location.lat(), 6) +
                      ",\"lng\":" + String(gps.location.lng(), 6) +
                      ",\"valid\":" + String(gps.location.isValid()) +
                      ",\"sats\":" + String(gps.satellites.value()) +
                      ",\"fix\":\"" + String(gps.location.isValid() ? "Active" : "No Fix") + "\"" +
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
        autonomousMode = true;  // Enable autonomous mode
        
        server.send(200, "text/plain", "Navigation started");
    });

    server.on("/stopNavigation", HTTP_GET, []() {
        autonomousMode = false;  // Disable autonomous mode
        setMotorSpeed(0, 0);    // Stop the car
        setSteeringAngle(90);   // Center the steering
        server.send(200, "text/plain", "Navigation stopped");
    });

    server.begin();
    Serial.println("HTTP server started");
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

float getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH);
    float distance = duration * 0.034 / 2; // Convert to cm
    
    return distance;
}

void loop() {
    // Get distances from ultrasonic sensors
    float distanceLeft = getDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
    float distanceCenter = getDistance(TRIG_PIN_CENTER, ECHO_PIN_CENTER);
    float distanceRight = getDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);

    // Obstacle avoidance
    if (distanceCenter < 300 || distanceLeft < 200 || distanceRight < 200) {
        if (distanceCenter < 300) { // If obstacle in front
            if (distanceLeft > distanceRight) {
                // If more space on the left, steer left
                setSteeringAngle(60); // Adjust angle as needed
                setMotorSpeed(100, 100); // Move forward
            } else {
                // If more space on the right, steer right
                setSteeringAngle(120); // Adjust angle as needed
                setMotorSpeed(100, 100); // Move forward
            }
        } else if (distanceLeft < 200) { // If obstacle on the left
            // Steer right
            setSteeringAngle(120); // Adjust angle 
            setMotorSpeed(100, 100); // Move forward
        } else if (distanceRight < 200) { // If obstacle on the right
            // Steer left
            setSteeringAngle(60); // Adjust angle 
            setMotorSpeed(100, 100); // Move forward
        }
    } else if (autonomousMode && gps.location.isValid()) {
        // Autonomous navigation logic
        float currentLat = gps.location.lat();
        float currentLon = gps.location.lng();
        
        float distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
        float bearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
        
        // Get current heading from GPS if available
        float currentHeading = gps.course.deg();
        
        // Calculate the steering angle needed
        float headingError = bearing - currentHeading;
        // Normalize to -180 to 180
        if (headingError > 180) headingError -= 360;
        if (headingError < -180) headingError += 360;
        
        // Convert heading error to steering angle (90 is center)
        int steeringAngle = 90 + (headingError * 0.5); // Scale factor of 0.5 to dampen response
        steeringAngle = constrain(steeringAngle, 55, 125); // Limit steering range
        
        // Set speed based on distance and turn severity
        int speed = 100;  // Base speed
        if (distance < 2.0) {  // If within 2 meters
            speed = 0;  // Stop at destination
            autonomousMode = false;  // Turn off autonomous mode
        } else if (abs(headingError) > 90) {
            speed = 50;  // Slow down for sharp turns
        }
        
        // Apply controls
        setSteeringAngle(steeringAngle);
        setMotorSpeed(speed, speed);
        
        // Debug output
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.print("m, Bearing: ");
        Serial.print(bearing);
        Serial.print("°, Heading Error: ");
        Serial.print(headingError);
        Serial.print("°, Steering Angle: ");
        Serial.println(steeringAngle);
    }

    // Handle client requests
    server.handleClient();

    // Check if the timeout period has elapsed
    if (millis() - lastUpdateTime > TIMEOUT_MS) {
        // Stop the motors and reset the steering angle
        setMotorSpeed(0, 0);  // Stop the car
        setSteeringAngle(90);  // Center the steering
    }
}

// Helper function to calculate distance between two coordinates (Haversine formula)
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    // Convert to radians
    float lat1Rad = lat1 * PI / 180.0;
    float lon1Rad = lon1 * PI / 180.0;
    float lat2Rad = lat2 * PI / 180.0;
    float lon2Rad = lon2 * PI / 180.0;
    
    // Haversine formula
    float dLat = lat2Rad - lat1Rad;
    float dLon = lon2Rad - lon1Rad;
    float a = sin(dLat/2) * sin(dLat/2) +
              cos(lat1Rad) * cos(lat2Rad) * 
              sin(dLon/2) * sin(dLon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    // Earth's radius in meters
    float r = 6371000;
    return c * r;  // Returns distance in meters
}

// Helper function to calculate bearing between two coordinates
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    // Convert to radians
    float lat1Rad = lat1 * PI / 180.0;
    float lon1Rad = lon1 * PI / 180.0;
    float lat2Rad = lat2 * PI / 180.0;
    float lon2Rad = lon2 * PI / 180.0;
    
    float dLon = lon2Rad - lon1Rad;
    
    float y = sin(dLon) * cos(lat2Rad);
    float x = cos(lat1Rad) * sin(lat2Rad) -
              sin(lat1Rad) * cos(lat2Rad) * cos(dLon);
    
    float bearing = atan2(y, x);
    bearing = bearing * 180 / PI;  // Convert to degrees
    bearing = fmod((bearing + 360), 360);  // Normalize to 0-360
    
    return bearing;
}
