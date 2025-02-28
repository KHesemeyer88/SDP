#ifndef WEBPAGE_H
#define WEBPAGE_H

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

        <!-- New manual mode GPS data section -->
        <div id="manual-gps-data" style="margin-top: 20px; background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); text-align: center;">
            <p>Fix Status: <span id="manual-fix">No Fix</span></p>
            <p>Latitude: <span id="manual-lat">--</span></p>
            <p>Longitude: <span id="manual-lng">--</span></p>
            <button id="fusion-status-btn" class="submit-btn" style="background-color: #007bff;" onclick="getFusionStatus()">Get Fusion Status</button>
            <p id="fusion-status-display" style="margin-top: 10px; display: none;">Fusion Status: <span id="fusion-status">--</span></p>
            <p id="waypoint-count" style="margin-top: 10px;">Waypoints: 0/20</p>
            <div style="display: flex; gap: 10px; justify-content: center;">
                <button id="record-waypoint-btn" class="submit-btn" style="background-color: #28a745;" onclick="recordWaypoint()">Record WP</button>
                <button id="clear-waypoint-btn" class="submit-btn" style="background-color: #dc3545;" onclick="clearWaypoints()">Clear WP</button>
            </div>
            <p id="recorded-waypoint-display" style="margin-top: 10px; display: none;">Latest WP: <span id="recorded-waypoint">--</span></p>
        </div>

        <div id="autonomous-control">
            <h2>Set Destination or Waypoints</h2>
            <input type="text" id="coords-input" class="coordinate-input" placeholder="Coordinates (e.g. 42.637088, -72.729328)">
            
            <div style="margin-top: 20px; display: flex; flex-wrap: wrap; justify-content: center; gap: 10px;">
                <div style="flex: 1; min-width: 250px; max-width: 350px;">
                    <h3>Advanced Settings</h3>
                    <div style="margin-bottom: 10px;">
                        <label for="loop-count">Waypoint Loop Count:</label>
                        <input type="number" id="loop-count" class="coordinate-input" value="1" min="1" max="100" style="width: 80px;">
                    </div>
                    <div style="margin-bottom: 10px;">
                        <label for="target-pace">Target Pace (m/s):</label>
                        <input type="number" id="target-pace" class="coordinate-input" value="0" min="0" step="0.1" style="width: 80px;">
                        <span style="font-size: 14px; color: #666;">0 = no pace control</span>
                    </div>
                    <div style="margin-bottom: 10px;">
                        <label for="target-distance">Target Distance (m):</label>
                        <input type="number" id="target-distance" class="coordinate-input" value="0" min="0" style="width: 80px;">
                        <span style="font-size: 14px; color: #666;">0 = no distance limit</span>
                    </div>
                </div>
                
                <div style="flex: 1; min-width: 250px; max-width: 350px;">
                    <h3>Navigation Status</h3>
                    <div style="background: #f0f0f0; padding: 10px; border-radius: 4px; margin-bottom: 10px;">
                        <p>Total Distance: <span id="total-distance">0.0</span> m</p>
                        <p>Current Pace: <span id="current-pace">0.0</span> m/s</p>
                        <p>Elapsed Time: <span id="elapsed-time">00:00:00</span></p>
                        <p>Loop: <span id="current-loop">0</span>/<span id="target-loops">1</span></p>
                    </div>
                </div>
            </div>
            
            <div style="margin-top: 20px;">
                <button class="submit-btn" onclick="startAutonomousMode()">Start Navigation</button>
                <button class="submit-btn" style="background-color: #dc3545;" onclick="stopAutonomousMode()">Stop Navigation</button>
            </div>
            
            <div id="avoidance-alert" style="display: none; margin-top: 10px; padding: 10px; background-color: #ffc107; border-radius: 4px;"></div>
        </div>
        <!-- Change existing gps-data div to be autonomous-specific -->
        <div id="autonomous-gps-data" style="display: none;">
            <p>Distance to Target: <span id="distance">--</span> m</p>
            <p>Current Bearing: <span id="bearing">--</span>&deg</p>
            <p>Fix Status: <span id="auto-fix">No Fix</span></p>
            <p>Target Location:</p>
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
                document.getElementById('manual-gps-data').style.display = 'block';
                document.getElementById('autonomous-gps-data').style.display = 'none';
                stopAutonomousMode();
            } else {
                document.getElementById('manual-control').style.display = 'none';
                document.getElementById('autonomous-control').style.display = 'block';
                document.getElementById('manual-btn').className = 'inactive';
                document.getElementById('auto-btn').className = 'active';
                document.getElementById('manual-gps-data').style.display = 'none';
                document.getElementById('autonomous-gps-data').style.display = 'block';
            }
        }

        function startAutonomousMode() {
            const coordsInput = document.getElementById('coords-input').value.trim();
            const loopCount = document.getElementById('loop-count').value;
            const targetPace = document.getElementById('target-pace').value;
            const targetDistance = document.getElementById('target-distance').value;
            
            // Update display of target loops
            document.getElementById('target-loops').textContent = loopCount;
            
            // Construct the URL with the new parameters
            let url = '/setDestination';
            const params = [];
            
            if (coordsInput) {
                // If coordinates are manually entered, validate and use them
                const coords = coordsInput.split(',').map(coord => coord.trim());
                
                if (coords.length !== 2 || isNaN(coords[0]) || isNaN(coords[1])) {
                    alert('Please enter valid coordinates or clear input to use recorded waypoint');
                    return;
                }
                
                params.push(`lat=${coords[0]}`);
                params.push(`lng=${coords[1]}`);
            }
            
            // Add the new parameters
            params.push(`loops=${loopCount}`);
            params.push(`pace=${targetPace}`);
            params.push(`distance=${targetDistance}`);
            
            // Construct the full URL
            if (params.length > 0) {
                url += '?' + params.join('&');
            }
            
            fetch(url)
                .then(response => response.text())
                .then(result => {
                    console.log('Navigation started:', result);
                    // Reset displays
                    document.getElementById('total-distance').textContent = '0.0';
                    document.getElementById('current-pace').textContent = '0.0';
                    document.getElementById('elapsed-time').textContent = '00:00:00';
                    document.getElementById('current-loop').textContent = '0';
                })
                .catch(error => {
                    console.error('Error starting navigation:', error);
                    alert('Failed to start navigation');
                });
        }

        // function to format time as HH:MM:SS
        function formatTime(ms) {
            const totalSeconds = Math.floor(ms / 1000);
            const hours = Math.floor(totalSeconds / 3600);
            const minutes = Math.floor((totalSeconds % 3600) / 60);
            const seconds = totalSeconds % 60;
            
            return String(hours).padStart(2, '0') + ':' +
                  String(minutes).padStart(2, '0') + ':' +
                  String(seconds).padStart(2, '0');
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
                    // Update manual mode display
                    document.getElementById('manual-fix').textContent = data.fix;
                    document.getElementById('manual-lat').textContent = data.lat || '--';
                    document.getElementById('manual-lng').textContent = data.lng || '--';

                    // Update autonomous mode display
                    if (document.getElementById('autonomous-control').style.display !== 'none') {
                        document.getElementById('distance').textContent = data.distance;
                        document.getElementById('bearing').textContent = data.bearing;
                        document.getElementById('auto-fix').textContent = data.fix;
                        document.getElementById('dest-lat').textContent = data.destLat;
                        document.getElementById('dest-lng').textContent = data.destLng;
                    }
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
        }, 500);  // Update every 500ms

        // Endpoint to fetch navigation stats
        setInterval(() => {
            if (document.getElementById('autonomous-control').style.display !== 'none') {
                fetch('/navstats')
                    .then(response => response.json())
                    .then(data => {
                        document.getElementById('total-distance').textContent = data.totalDistance.toFixed(1);
                        document.getElementById('current-pace').textContent = data.currentPace.toFixed(2);
                        document.getElementById('elapsed-time').textContent = formatTime(data.totalTime);
                        document.getElementById('current-loop').textContent = data.currentLoop;
                        document.getElementById('target-loops').textContent = data.targetLoops;
                    })
                    .catch(console.error);
            }
        }, 1000);



        function getFusionStatus() {
            const button = document.getElementById('fusion-status-btn');
            const display = document.getElementById('fusion-status-display');
            const statusSpan = document.getElementById('fusion-status');
            
            // Disable button while fetching
            button.disabled = true;
            button.textContent = 'Getting Status...';

            // Show display immediately
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
                    // Ensure display stays visible
                    display.style.display = 'block';
                });
        }
        function recordWaypoint() {
            const button = document.getElementById('record-waypoint-btn');
            const display = document.getElementById('recorded-waypoint-display');
            const waypointSpan = document.getElementById('recorded-waypoint');
            const countDisplay = document.getElementById('waypoint-count');
            
            // Disable button while fetching
            button.disabled = true;
            button.textContent = 'Recording WP...';

            // Show display immediately
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
                    // Ensure display stays visible
                    display.style.display = 'block';
                });
        }

        function clearWaypoints() {
            const button = document.getElementById('clear-waypoint-btn');
            const countDisplay = document.getElementById('waypoint-count');
            
            // Disable button while clearing
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