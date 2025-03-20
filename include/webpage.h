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
        .connection-status {
            position: fixed;
            top: 10px;
            right: 10px;
            padding: 5px 10px;
            border-radius: 4px;
            font-size: 14px;
            color: white;
        }
        .connected {
            background-color: #28a745;
        }
        .disconnected {
            background-color: #dc3545;
        }
    </style>
</head>
<body>
    <div id="connection-status" class="connection-status disconnected">Disconnected</div>
    
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
                    <button id="start-pause-btn" class="submit-btn" onclick="toggleStartPause()">Start</button>
                    <button id="stop-btn" class="submit-btn" style="background-color: #dc3545;" onclick="stopAutonomousMode()">Stop</button>
                    <button id="reset-tracking-btn" class="submit-btn" style="background-color: #ffc107;" onclick="resetTracking()">Reset Tracking</button>
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
    // WebSocket connection
    let ws;
    let wsConnected = false;
    let reconnectInterval;
    const WS_RECONNECT_INTERVAL = 3000; // 3 seconds between reconnect attempts
    const CONNECTION_TIMEOUT = 10000; // 10 seconds timeout for connection attempts

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

    // Global variable to track navigation state
    let navigationActive = false;
    let navigationPaused = false;

    // Canvas drawing function
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

    // Mode switching
    function switchMode(mode) {
        if (mode === 'manual') {
            document.getElementById('manual-control').style.display = 'flex';
            document.getElementById('autonomous-control').style.display = 'none';
            document.getElementById('manual-btn').className = 'active';
            document.getElementById('auto-btn').className = 'inactive';
            document.getElementById('manual-gps-data').style.display = 'block';
            stopAutonomousMode();
        } else {
            document.getElementById('manual-control').style.display = 'none';
            document.getElementById('autonomous-control').style.display = 'block';
            document.getElementById('manual-btn').className = 'inactive';
            document.getElementById('auto-btn').className = 'active';
            document.getElementById('manual-gps-data').style.display = 'none';
        }
    }

    // Toggle between start and pause
    function toggleStartPause() {
        if (!navigationActive) {
            // Start navigation
            startAutonomousMode();
        } else if (navigationPaused) {
            // Resume navigation
            resumeNavigation();
        } else {
            // Pause navigation
            pauseNavigation();
        }
    }

    // Start autonomous navigation
    function startAutonomousMode() {
        if (!wsConnected) {
            showAlert("WebSocket disconnected. Cannot start autonomous mode.");
            return;
        }

        const coordsInput = document.getElementById('coords-input').value.trim();
        const targetPace = document.getElementById('target-pace').value;
        const targetDistance = document.getElementById('target-distance').value;
        
        // Build command object
        const command = {
            autonomous: "start",
            pace: parseFloat(targetPace),
            distance: parseFloat(targetDistance)
        };
        
        // Add coordinates if provided
        if (coordsInput) {
            const coords = coordsInput.split(',').map(coord => coord.trim());
            if (coords.length !== 2 || isNaN(coords[0]) || isNaN(coords[1])) {
                showAlert('Please enter valid coordinates or clear input to use recorded waypoint');
                return;
            }
            command.lat = parseFloat(coords[0]);
            command.lng = parseFloat(coords[1]);
        }
        
        // Send command via WebSocket
        ws.send(JSON.stringify(command));
        
        // Update UI state
        navigationActive = true;
        navigationPaused = false;
        document.getElementById('start-pause-btn').textContent = 'Pause';
        document.getElementById('start-pause-btn').style.backgroundColor = '#ffc107';
        
        // Reset UI elements
        document.getElementById('total-distance').textContent = '0.0';
        document.getElementById('current-pace').textContent = '0.0';
        document.getElementById('average-pace').textContent = '0.0';
        document.getElementById('elapsed-time').textContent = '00:00:00';
    }

    // Pause navigation
    function pauseNavigation() {
        if (!wsConnected) {
            showAlert("WebSocket disconnected. Cannot pause navigation.");
            return;
        }
        
        ws.send(JSON.stringify({ autonomous: "pause" }));
        navigationPaused = true;
        document.getElementById('start-pause-btn').textContent = 'Resume';
        document.getElementById('start-pause-btn').style.backgroundColor = '#28a745';
    }

    // Resume navigation
    function resumeNavigation() {
        if (!wsConnected) {
            showAlert("WebSocket disconnected. Cannot resume navigation.");
            return;
        }
        
        ws.send(JSON.stringify({ autonomous: "resume" }));
        navigationPaused = false;
        document.getElementById('start-pause-btn').textContent = 'Pause';
        document.getElementById('start-pause-btn').style.backgroundColor = '#ffc107';
    }
        
    // Stop autonomous navigation
    function stopAutonomousMode() {
        if (!wsConnected) {
            showAlert("WebSocket disconnected. Cannot stop autonomous mode.");
            return;
        }
        
        ws.send(JSON.stringify({ autonomous: "stop" }));
        
        // Update UI state
        navigationActive = false;
        navigationPaused = false;
        document.getElementById('start-pause-btn').textContent = 'Start';
        document.getElementById('start-pause-btn').style.backgroundColor = '#28a745';
    }

    // Reset tracking data
    function resetTracking() {
        if (wsConnected) {
            ws.send(JSON.stringify({ tracking: "reset" }));
        } else {
            showAlert("WebSocket disconnected. Cannot reset tracking.");
        }
    }

    // Format time for display
    function formatTime(ms) {
        const totalSeconds = Math.floor(ms / 1000);
        const hours = Math.floor(totalSeconds / 3600);
        const minutes = Math.floor((totalSeconds % 3600) / 60);
        const seconds = totalSeconds % 60;
        return String(hours).padStart(2, '0') + ':' +
              String(minutes).padStart(2, '0') + ':' +
              String(seconds).padStart(2, '0');
    }

    // Update RTK status display
    function updateRTKDisplay(data) {
        document.getElementById('rtk-correction-status').textContent = data.status;
        document.getElementById('rtk-correction-age').textContent = data.age;
        document.getElementById('rtk-connected').textContent = data.connected ? 'Connected' : 'Disconnected';
        
        // Display RTK solution status
        if (data.carrSoln !== undefined) {
            let solutionText = 'None';
            if (data.carrSoln === 1) solutionText = 'Float';
            if (data.carrSoln === 2) solutionText = 'Fixed';
            document.getElementById('rtk-solution').textContent = solutionText;
        }
        
        // Display horizontal accuracy (hAcc)
        if (data.hAcc !== undefined) {
            document.getElementById('rtk-hacc').textContent = data.hAcc.toFixed(2);
        }
        
        // Display fix type
        if (data.fixType !== undefined) {
            let fixTypeText = 'No Fix';
            if (data.fixType === 1) fixTypeText = '1 (Dead Reckoning)';
            if (data.fixType === 2) fixTypeText = '2 (2D)';
            if (data.fixType === 3) fixTypeText = '3 (3D)';
            if (data.fixType === 4) fixTypeText = '4 (GNSS + Dead Reckoning)';
            if (data.fixType === 5) fixTypeText = '5 (Time Only)';
            document.getElementById('rtk-fix-type').textContent = fixTypeText;
        }
        
        // Color coding for different statuses
        const statusElement = document.getElementById('rtk-correction-status');
        if (data.status === 'Fresh') {
            statusElement.style.color = 'green';
        } else if (data.status === 'Stale') {
            statusElement.style.color = 'orange';
        } else {
            statusElement.style.color = 'red';
        }
        
        // Color coding for RTK solution
        const solutionElement = document.getElementById('rtk-solution');
        if (data.carrSoln === 2) {
            solutionElement.style.color = 'green'; // Fixed solution
        } else if (data.carrSoln === 1) {
            solutionElement.style.color = 'orange'; // Float solution
        } else {
            solutionElement.style.color = 'red'; // No RTK solution
        }
    }

    // Show an alert message
    function showAlert(message) {
        const alert = document.getElementById('avoidance-alert');
        alert.textContent = message;
        alert.style.display = 'block';
        setTimeout(() => {
            alert.style.display = 'none';
        }, 3000);
    }

    // Update connection status indicator
    function updateConnectionStatus(connected) {
        const statusElement = document.getElementById('connection-status');
        if (connected) {
            statusElement.textContent = 'Connected';
            statusElement.className = 'connection-status connected';
        } else {
            statusElement.textContent = 'Disconnected';
            statusElement.className = 'connection-status disconnected';
        }
    }

    // Event listeners for joystick
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

    // Initialize websocket when DOM is ready
    document.addEventListener('DOMContentLoaded', function() {
        initWebSocket();
        drawJoystick();
    });

    // Handle mouse movement for joystick
    function handleMouseMove(e) {
        const rect = canvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;
        updateJoystickPosition(mouseX, mouseY);
    }

    // Handle touch movement for joystick
    function handleTouchMove(e) {
        const touch = e.touches[0];
        const rect = canvas.getBoundingClientRect();
        const touchX = touch.clientX - rect.left;
        const touchY = touch.clientY - rect.top;
        updateJoystickPosition(touchX, touchY);
    }

    // Handle end of input for joystick
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

    // Update joystick position
    function updateJoystickPosition(x, y) {
        const dx = Math.min(Math.max(x - centerX, -joystickSize), joystickSize);
        const dy = Math.min(Math.max(y - centerY, -joystickSize), joystickSize);
        handleX = centerX + dx;
        handleY = centerY + dy;
        drawJoystick();
    }

    // Calculate values from joystick position
    function calculateValues() {
        const dx = handleX - centerX;
        const dy = handleY - centerY;
        const maxDistance = joystickSize;
        
        // Normalize to -1 to 1
        const normalizedX = dx / maxDistance;
        const normalizedY = -dy / maxDistance;
        
        return { normalizedY, normalizedX };
    }
    
    // Send joystick update
    function sendUpdate() {
        if (!wsConnected) {
            return;
        }

        const { normalizedY, normalizedX } = calculateValues();
        
        ws.send(JSON.stringify({
            control: {
                vertical: normalizedY,
                horizontal: normalizedX
            }
        }));
    }

    // Get fusion status from ESP32
    function getFusionStatus() {
        const button = document.getElementById('fusion-status-btn');
        const display = document.getElementById('fusion-status-display');
        const statusSpan = document.getElementById('fusion-status');
        
        button.disabled = true;
        button.textContent = 'Getting Status...';
        display.style.display = 'block';
        
        // For fusion status, we'll use HTTP since it requires special handling
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

    // Record waypoint
    function recordWaypoint() {
        if (!wsConnected) {
            showAlert("WebSocket disconnected. Cannot record waypoint.");
            return;
        }

        const button = document.getElementById('record-waypoint-btn');
        button.disabled = true;
        button.textContent = 'Recording WP...';
        
        ws.send(JSON.stringify({ waypoint: "record" }));
        
        // Always re-enable the button after a short delay
        setTimeout(() => {
            button.disabled = false;
            button.textContent = 'Record WP';
        }, 1000);
    }

    // Clear all waypoints
    function clearWaypoints() {
        if (!wsConnected) {
            showAlert("WebSocket disconnected. Cannot clear waypoints.");
            return;
        }

        const button = document.getElementById('clear-waypoint-btn');
        button.disabled = true;
        button.textContent = 'Clearing...';
        
        ws.send(JSON.stringify({ waypoint: "clear" }));
        
        // Always re-enable the button after a short delay
        setTimeout(() => {
            button.disabled = false;
            button.textContent = 'Clear WP';
        }, 1000);
    }

    // Initialize WebSocket connection
    function initWebSocket() {
        // Get current location host and use it to build WebSocket URL
        const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${wsProtocol}//${window.location.hostname}:81`;
        
        console.log(`Connecting to WebSocket at ${wsUrl}`);
        
        // Clear any existing reconnect interval
        if (reconnectInterval) {
            clearInterval(reconnectInterval);
            reconnectInterval = null;
        }
        
        // Create new WebSocket connection
        ws = new WebSocket(wsUrl);
        
        // Set connection timeout
        const connectionTimeout = setTimeout(() => {
            if (!wsConnected) {
                console.log('WebSocket connection timeout');
                ws.close();
            }
        }, CONNECTION_TIMEOUT);
        
        // WebSocket event handlers
        ws.onopen = function(evt) {
            console.log('WebSocket connected');
            wsConnected = true;
            clearTimeout(connectionTimeout);
            updateConnectionStatus(true);
        };
        
        ws.onclose = function(evt) {
            console.log('WebSocket disconnected');
            wsConnected = false;
            updateConnectionStatus(false);
            
            // Start reconnection attempts
            if (!reconnectInterval) {
                reconnectInterval = setInterval(initWebSocket, WS_RECONNECT_INTERVAL);
            }
        };
        
        ws.onerror = function(evt) {
            console.error('WebSocket error:', evt);
        };
        
        ws.onmessage = function(evt) {
            try {
                const data = JSON.parse(evt.data);
                
                // Handle different message types
                switch(data.type) {
                    case "sensors":
                        // Update sensor displays
                        document.getElementById('sensor-readout').textContent = 
                            `Front: ${data.front} cm | Left: ${data.left} cm | Right: ${data.right} cm`;
                        
                        if (data.message && data.message !== "") {
                            showAlert(data.message);
                        }
                        break;
                        
                    case "gps":
                        // Update GPS displays
                        document.getElementById('manual-fix').textContent = data.fix;
                        document.getElementById('manual-lat').textContent = data.lat || '--';
                        document.getElementById('manual-lng').textContent = data.lng || '--';
                        break;
                        
                    case "rtk":
                        // Update RTK status
                        updateRTKDisplay(data);
                        break;
                        
                    case "navstats":
                        // Update navigation stats
                        document.getElementById('total-distance').textContent = data.totalDistance.toFixed(1);
                        document.getElementById('current-pace').textContent = data.currentPace.toFixed(2);
                        document.getElementById('average-pace').textContent = data.averagePace.toFixed(2);
                        document.getElementById('elapsed-time').textContent = formatTime(data.totalTime);
                        break;
                        
                    case "waypoint":
                        // Update waypoint info
                        document.getElementById('waypoint-count').textContent = `Waypoints: ${data.count}/20`;
                        if (data.lat && data.lng) {
                            document.getElementById('recorded-waypoint').textContent = data.lat + ", " + data.lng;
                            document.getElementById('recorded-waypoint-display').style.display = 'block';
                        }
                        break;
                        
                    case "status":
                        // Show status message
                        showAlert(data.message);
                        break;
                        
                    case "error":
                        // Show error message
                        console.error("Error received:", data.message);
                        showAlert(data.message);
                        break;
                }
            } catch (e) {
                console.error('Error parsing WebSocket message:', e);
            }
        };
    }
</script>
</body>
</html>
)rawliteral";

#endif // WEBPAGE_H