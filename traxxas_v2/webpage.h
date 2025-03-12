#ifndef WEBPAGE_H
#define WEBPAGE_H
#include "config.h"

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
            <button id="manual-btn" class="active" onclick="switchMode('manual')">Manual</button>
            <button id="auto-btn" class="inactive" onclick="switchMode('autonomous')">Autonomous</button>
        </div>

        <div id="manual-control">
            <div id="sensor-readout" style="font-size: 14px; margin-bottom: 10px; font-family: monospace;">
                Front: -- cm | Left: -- cm | Right: -- cm
            </div>
            <canvas id="joystick" width="300" height="300"></canvas>
        </div>

        <div id="manual-gps-data" style="margin-top: 20px; background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); text-align: center;">
            <div style="display: flex; gap: 10px; justify-content: center;">
                <button id="record-waypoint-btn" class="submit-btn" style="background-color: #28a745;" onclick="recordWaypoint()">Record WP</button>
                <button id="clear-waypoint-btn" class="submit-btn" style="background-color: #dc3545;" onclick="clearWaypoints()">Clear WP</button>
            </div>
            <p>Fix Status: <span id="manual-fix">No Fix</span></p>
            <p>Latitude: <span id="manual-lat">--</span></p>
            <p>Longitude: <span id="manual-lng">--</span></p>
            <p id="waypoint-count" style="margin-top: 10px;">Waypoints: 0/20</p>
            <p id="recorded-waypoint-display" style="margin-top: 10px; display: none;">Latest WP: <span id="recorded-waypoint">--</span></p>
            <button id="fusion-status-btn" class="submit-btn" style="background-color: #007bff;" onclick="getFusionStatus()">Get Fusion Status</button>
            <p id="fusion-status-display" style="margin-top: 10px; display: none;">Fusion Status: <span id="fusion-status">--</span></p>
        </div>

        <div id="autonomous-control">          
            <div style="margin-top: 5px; display: flex; flex-wrap: wrap; justify-content: center; gap: 5px;">
                <div style="flex: 1; min-width: 250px; max-width: 350px;">
                    <div style="margin-bottom: 5px;">
                        <label for="target-pace">Pace (m/s):</label>
                        <input type="number" id="target-pace" class="coordinate-input" value="0" min="0" step="0.1" style="width: 80px;">
                    </div>
                    <div style="margin-bottom: 5px;">
                        <label for="target-distance">Distance (m):</label>
                        <input type="number" id="target-distance" class="coordinate-input" value="0" min="0" style="width: 80px;">
                    </div>
                    <button class="submit-btn" onclick="startAutonomousMode()">Start</button>
                    <button class="submit-btn" style="background-color: #dc3545;" onclick="stopAutonomousMode()">Stop</button>
                    <button class="submit-btn" style="background-color: #ffc107;" onclick="resetTracking()">Reset Tracking</button>
                </div>
                <div style="flex: 1; min-width: 250px; max-width: 350px;">
                    <div style="background: #f0f0f0; padding: 10px; border-radius: 4px; margin-bottom: 10px;">
                        <p>Distance: <span id="total-distance">0.0</span> m</p>
                        <p>Instantaneous Pace: <span id="current-pace">0.0</span> m/s</p>
                        <p>Average Pace: <span id="average-pace">0.0</span> m/s</p>
                        <p>Time: <span id="elapsed-time">00:00:00</span></p>
                    </div>
                </div>
                <div id="rtk-status-panel" style="margin-top: 20px; background: white; padding: 10px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); text-align: center;">
                    <p>Correction Status: <span id="rtk-correction-status">Unknown</span></p>
                    <p>Correction Age: <span id="rtk-correction-age">--</span> ms</p>
                    <p>RTK Connection: <span id="rtk-connected">--</span></p>
                    <p>RTK Solution: <span id="rtk-solution">--</span></p>
                    <p>Horizontal Accuracy: <span id="rtk-hacc">--</span> cm</p>
                    <p>Fix Type: <span id="rtk-fix-type">--</span></p>
                </div>
            </div>
                        
            <div id="avoidance-alert" style="display: none; margin-top: 10px; padding: 10px; background-color: #ffc107; border-radius: 4px;"></div>
            <h3>Input Coords</h3>
            <input type="text" id="coords-input" class="coordinate-input" placeholder="Coordinates (e.g. 42.637088, -72.729328)">
        </div>        
    </div>

<script>
        const canvas = document.getElementById('joystick');
        const ctx = canvas.getContext('2d');
        const joystickSize = 150;
        const handleRadius = 30;
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;

        let sensorPollingInterval = null;

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
                document.getElementById('manual-gps-data').style.display = 'block';
                stopAutonomousMode();
                // Start sensor polling in manual mode
                if (sensorPollingInterval === null) {
                    sensorPollingInterval = setInterval(updateSensorReadings, 500);
                }
            } else {
                document.getElementById('manual-control').style.display = 'none';
                document.getElementById('autonomous-control').style.display = 'block';
                document.getElementById('manual-btn').className = 'inactive';
                document.getElementById('auto-btn').className = 'active';
                document.getElementById('manual-gps-data').style.display = 'none';
                // Stop sensor polling in autonomous mode
                if (sensorPollingInterval !== null) {
                    clearInterval(sensorPollingInterval);
                    sensorPollingInterval = null;
                }
            }
        }

        function startAutonomousMode() {
            const coordsInput = document.getElementById('coords-input').value.trim();
            const targetPace = document.getElementById('target-pace').value;
            const targetDistance = document.getElementById('target-distance').value;
            
            let url = '/setDestination';
            const params = [];
            
            if (coordsInput) {
                const coords = coordsInput.split(',').map(coord => coord.trim());
                if (coords.length !== 2 || isNaN(coords[0]) || isNaN(coords[1])) {
                    alert('Please enter valid coordinates or clear input to use recorded waypoint');
                    return;
                }
                params.push(`lat=${coords[0]}`);
                params.push(`lng=${coords[1]}`);
            }
            
            params.push(`pace=${targetPace}`);
            params.push(`distance=${targetDistance}`);
            
            if (params.length > 0) {
                url += '?' + params.join('&');
            }
            
            fetch(url)
                .then(response => response.text())
                .then(result => {
                    console.log('Navigation started:', result);
                    document.getElementById('total-distance').textContent = '0.0';
                    document.getElementById('current-pace').textContent = '0.0';
                    document.getElementById('average-pace').textContent = '0.0';
                    document.getElementById('elapsed-time').textContent = '00:00:00';
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

        function resetTracking() {
            fetch('/resetTracking')
                .then(response => response.text())
                .then(result => {
                    console.log('Tracking reset:', result);
                })
                .catch(error => {
                    console.error('Error resetting tracking:', error);
                });
        }

        function formatTime(ms) {
            const totalSeconds = Math.floor(ms / 1000);
            const hours = Math.floor(totalSeconds / 3600);
            const minutes = Math.floor((totalSeconds % 3600) / 60);
            const seconds = totalSeconds % 60;
            return String(hours).padStart(2, '0') + ':' +
                  String(minutes).padStart(2, '0') + ':' +
                  String(seconds).padStart(2, '0');
        }

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
            
            // Normalize to -1 to 1
            const normalizedX = dx / maxDistance;
            const normalizedY = -dy / maxDistance;
            
            return { normalizedY, normalizedX };
        }

        function sendUpdate() {
            const { normalizedY, normalizedX } = calculateValues();
            fetch(`/control?vertical=${normalizedY}&horizontal=${normalizedX}`)
                .catch((e) => console.error('Error:', e));
        }

        function updateSensorReadings() {
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
                        }, 3000);
                    }
                })
                .catch(console.error);
        }

        // Start sensor polling only in manual mode (initial state)
        sensorPollingInterval = setInterval(updateSensorReadings, 500);

        drawJoystick();

        setInterval(() => {
            fetch('/gps')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('manual-fix').textContent = data.fix;
                    document.getElementById('manual-lat').textContent = data.lat || '--';
                    document.getElementById('manual-lng').textContent = data.lng || '--';
                })
                .catch(console.error);
        }, 1000);

        // Keep polling for messages in autonomous mode
        setInterval(() => {
            if (document.getElementById('autonomous-control').style.display !== 'none') {
                fetch('/messages')  // New endpoint just for messages
                    .then(response => response.json())
                    .then(data => {
                        if (data.message && data.message !== "") {
                            const alert = document.getElementById('avoidance-alert');
                            alert.textContent = data.message;
                            alert.style.display = 'block';
                            setTimeout(() => {
                                alert.style.display = 'none';
                            }, 3000);
                        }
                    })
                    .catch(console.error);
            }
        }, 500);

        setInterval(() => {
            if (document.getElementById('autonomous-control').style.display !== 'none') {
                fetch('/navstats')
                    .then(response => response.json())
                    .then(data => {
                        document.getElementById('total-distance').textContent = data.totalDistance.toFixed(1);
                        document.getElementById('current-pace').textContent = data.currentPace.toFixed(2);
                        document.getElementById('average-pace').textContent = data.averagePace.toFixed(2);
                        document.getElementById('elapsed-time').textContent = formatTime(data.totalTime);
                    })
                    .catch(console.error);
            }
        }, 1000);

        // RTK status
        setInterval(() => {
            fetch('/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('rtk-correction-status').textContent = data.rtk.status;
                    document.getElementById('rtk-correction-age').textContent = data.rtk.age;
                    document.getElementById('rtk-connected').textContent = data.rtk.connected ? 'Connected' : 'Disconnected';
                    
                    // Display RTK solution status
                    if (data.rtk.carrSoln !== undefined) {
                        let solutionText = 'None';
                        if (data.rtk.carrSoln === 1) solutionText = 'Float';
                        if (data.rtk.carrSoln === 2) solutionText = 'Fixed';
                        document.getElementById('rtk-solution').textContent = solutionText;
                    }
                    
                    // Display horizontal accuracy (hAcc)
                    if (data.rtk.hAcc !== undefined) {
                        document.getElementById('rtk-hacc').textContent = data.rtk.hAcc.toFixed(2);
                    }
                    
                    // Display fix type
                    if (data.rtk.fixType !== undefined) {
                        let fixTypeText = 'No Fix';
                        if (data.rtk.fixType === 1) fixTypeText = '1 (Dead Reckoning)';
                        if (data.rtk.fixType === 2) fixTypeText = '2 (2D)';
                        if (data.rtk.fixType === 3) fixTypeText = '3 (3D)';
                        if (data.rtk.fixType === 4) fixTypeText = '4 (GNSS + Dead Reckoning)';
                        if (data.rtk.fixType === 5) fixTypeText = '5 (Time Only)';
                        document.getElementById('rtk-fix-type').textContent = fixTypeText;
                    }
                    
                    // Color coding for different statuses
                    const statusElement = document.getElementById('rtk-correction-status');
                    if (data.rtk.status === 'Fresh') {
                        statusElement.style.color = 'green';
                    } else if (data.rtk.status === 'Stale') {
                        statusElement.style.color = 'orange';
                    } else {
                        statusElement.style.color = 'red';
                    }
                    
                    // Color coding for RTK solution
                    const solutionElement = document.getElementById('rtk-solution');
                    if (data.rtk.carrSoln === 2) {
                        solutionElement.style.color = 'green'; // Fixed solution
                    } else if (data.rtk.carrSoln === 1) {
                        solutionElement.style.color = 'orange'; // Float solution
                    } else {
                        solutionElement.style.color = 'red'; // No RTK solution
                    }
                })
                .catch(console.error);
        }, 1000);

        function getFusionStatus() {
            const button = document.getElementById('fusion-status-btn');
            const display = document.getElementById('fusion-status-display');
            const statusSpan = document.getElementById('fusion-status');
            button.disabled = true;
            button.textContent = 'Getting Status...';
            display.style.display = 'block';
            
            fetch('/fusionStatus')
                .then(response => response.json())
                .then(data => {
                    statusSpan.textContent = data.status;
                    display.style.display = 'block';
                })
                .catch(error => {
                    statusSpan.textContent = 'Error getting status';
                    console.error('Error:', error);
                })
                .finally(() => {
                    button.disabled = false;
                    button.textContent = 'Get Fusion Status';
                    display.style.display = 'block';
                });
        }

        function recordWaypoint() {
            const button = document.getElementById('record-waypoint-btn');
            const display = document.getElementById('recorded-waypoint-display');
            const waypointSpan = document.getElementById('recorded-waypoint');
            const countDisplay = document.getElementById('waypoint-count');
            button.disabled = true;
            button.textContent = 'Recording WP...';
            display.style.display = 'block';
            
            fetch('/currentWP')
                .then(response => response.json())
                .then(data => {
                    waypointSpan.textContent = data.lat + ", " + data.lng;
                    countDisplay.textContent = `Waypoints: ${data.count}/20`;
                    display.style.display = 'block';
                })
                .catch(error => {
                    waypointSpan.textContent = 'Error getting WP';
                    console.error('Error:', error);
                })
                .finally(() => {
                    button.disabled = false;
                    button.textContent = 'Record WP';
                    display.style.display = 'block';
                });
        }

        function clearWaypoints() {
            const button = document.getElementById('clear-waypoint-btn');
            const countDisplay = document.getElementById('waypoint-count');
            button.disabled = true;
            button.textContent = 'Clearing...';
            
            fetch('/clearWaypoints')
                .then(response => response.json())
                .then(data => {
                    countDisplay.textContent = `Waypoints: ${data.count}/20`;
                    document.getElementById('recorded-waypoint-display').style.display = 'none';
                })
                .catch(error => {
                    console.error('Error:', error);
                })
                .finally(() => {
                    button.disabled = false;
                    button.textContent = 'Clear WP';
                });
        }
    </script>
</body>
</html>
)rawliteral";

#endif // WEBPAGE_H