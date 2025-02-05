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
//GPS

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
        canvas {
            background: #ddd;
            border: 2px solid #444;
            border-radius: 4px;
        }
        #gps-data {
            background: white;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            margin-top: 20px;
            width: 80%;
            text-align: center;
    </style>
</head>
<body>
    <canvas id="joystick" width="300" height="300"></canvas>
    <div id="gps-data" style="margin-top: 20px; font-family: sans-serif;">
        <p>Latitude: <span id="lat">--</span></p>
        <p>Longitude: <span id="lng">--</span></p>
        <p>Satellites: <span id="sats">--</span></p>
        <p>Fix Status: <span id="fix">No Fix</span></p>
    </div>
    <script>
        const canvas = document.getElementById('joystick');
        const ctx = canvas.getContext('2d');

        const joystickSize = 150; // Half the size of the square
        const handleRadius = 30;   // Handle radius
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;

        let handleX = centerX;
        let handleY = centerY;
        let isDragging = false;
        let updateInterval = null;  // Timer for periodic updates

        function drawJoystick() {
            ctx.clearRect(0, 0, canvas.width, canvas.height);

            // Draw square boundary
            ctx.strokeStyle = '#bbb';
            ctx.lineWidth = 4;
            ctx.strokeRect(centerX - joystickSize, centerY - joystickSize, joystickSize * 2, joystickSize * 2);

            // Draw handle
            ctx.beginPath();
            ctx.arc(handleX, handleY, handleRadius, 0, Math.PI * 2);
            ctx.fillStyle = '#444';
            ctx.fill();
        }

        function calculateValues() {
            const dx = handleX - centerX;
            const dy = handleY - centerY;
            const maxDistance = joystickSize;

            // Normalize X and Y to ranges
            const speed = Math.round((-dy / maxDistance) * 255); // Y-axis maps to speed
            const angle = Math.round((dx / maxDistance) * 35 + 90); // X-axis maps to angle

            return { speed, angle };
        }

        function sendUpdate() {
            const { speed, angle } = calculateValues();
            fetch(`/control?speed=${speed}&angle=${angle}`)
                .catch((e) => console.error(e));
        }

        canvas.addEventListener('touchstart', (e) => {
            isDragging = true;
            // Start sending updates periodically (e.g., every 100 ms)
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

            // Constrain to square bounds
            const dx = Math.min(Math.max(touchX - centerX, -joystickSize), joystickSize);
            const dy = Math.min(Math.max(touchY - centerY, -joystickSize), joystickSize);

            handleX = centerX + dx;
            handleY = centerY + dy;

            drawJoystick();
            // No need to call sendUpdate here since the timer is handling periodic updates
        });

        canvas.addEventListener('touchend', () => {
            isDragging = false;
            // Stop the periodic updates
            if (updateInterval !== null) {
                clearInterval(updateInterval);
                updateInterval = null;
            }
            // Reset the joystick to center and send a final update
            handleX = centerX;
            handleY = centerY;

            drawJoystick();
            sendUpdate(); // This sends the reset command immediately
        });

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
    //GPSSerial.begin(9600, SERIAL_8N1, 3, 1); // RX=3, TX=1
    Wire.begin(32,33);
    //Wire.setPins(2, 15);  // SDA, SCL
    Serial.println("\nI2C Scanner");

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
        String json = "{\"lat\":" + String(gps.location.lat(), 6) +
                      ",\"lng\":" + String(gps.location.lng(), 6) +
                      ",\"valid\":" + String(gps.location.isValid()) +
                      ",\"sats\":" + String(gps.satellites.value()) +
                      ",\"fix\":\"" + String(gps.location.isValid() ? "Active" : "No Fix") + "\"}";
        server.send(200, "application/json", json);
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

void loop() {
    //Serial.println("Loop running");
    //Serial.println("Scanning...");
    //for (byte address = 1; address < 127; address++) {
    //    Wire.beginTransmission(address);
    //    if (Wire.endTransmission() == 0) {
            //Serial.print("I2C device found at 0x");
            //Serial.println(address, HEX);
    //    }
    //}
    //delay(2000);
    Wire.requestFrom(0x42, 32);  // Request, say, 32 bytes from the GPS
    while (Wire.available()) {
      char c = Wire.read();
      //Serial.print(c);     // Optional: print the character to see raw data
      gps.encode(c);       // Pass the data to TinyGPS++
      //Serial.print("\n");
      //Serial.print("Satellites value:");
      //Serial.print(gps.satellites.value());
      //Serial.print("\n");
    }

    server.handleClient();

    // Check if the timeout period has elapsed
    if (millis() - lastUpdateTime > TIMEOUT_MS) {
        // Stop the motors and reset the steering angle
        setMotorSpeed(0, 0);  // Stop the car
        setSteeringAngle(90);  // Center the steering
    }
}
