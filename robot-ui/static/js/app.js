/**
 * Robot Control UI - Main Application
 * WebSocket communication and motor control logic
 */

// Global variables
let socket = null;
let currentVelocity = 0.3; // m/s
let isConnected = false;

// Motor command state
let activeDirection = null;
let pressedKeys = new Set(); // Track which keys are currently held down

// Odometry streaming state
let streamingEnabled = true;

// Occupancy map renderer
let occupancyMap = null;

// Initialize application
document.addEventListener('DOMContentLoaded', () => {
    console.log('Robot Control UI initializing...');
    initializeWebSocket();
    initializeControls();
    initializeKeyboardControls();
    initializeTabs();

    // Initialize occupancy map renderer (Phase 5)
    const mapCanvas = document.getElementById('map-canvas');
    if (mapCanvas) {
        occupancyMap = new OccupancyMapRenderer('map-canvas');
        console.log('✓ Occupancy map renderer initialized');
    }
});

/**
 * Tab Management
 */
function initializeTabs() {
    const tabButtons = document.querySelectorAll('.tab-btn');
    const tabContents = document.querySelectorAll('.tab-content');

    // Show first tab by default
    document.getElementById('tab-map-data').classList.add('active');

    tabButtons.forEach(button => {
        button.addEventListener('click', () => {
            const targetTab = button.getAttribute('data-tab');

            // Hide all tabs
            tabContents.forEach(content => {
                content.classList.remove('active');
                content.style.display = 'none';
            });

            // Remove active state from all buttons
            tabButtons.forEach(btn => {
                btn.classList.remove('tab-btn-active');
            });

            // Show selected tab
            const selectedTab = document.getElementById(`tab-${targetTab}`);
            if (selectedTab) {
                selectedTab.classList.add('active');
                selectedTab.style.display = 'grid';
            }

            // Add active state to clicked button
            button.classList.add('tab-btn-active');
        });
    });
}

/**
 * WebSocket Connection
 */
function initializeWebSocket() {
    // Connect to Flask-SocketIO server
    socket = io();

    socket.on('connect', () => {
        console.log('✓ Connected to WebSocket server');
        isConnected = true;
        // Don't update status here - wait for connection_status event from server
    });

    socket.on('disconnect', () => {
        console.log('✗ Disconnected from server');
        isConnected = false;
        updateConnectionStatus(false, 'Disconnected');
        stopMotor();
    });

    socket.on('connection_status', (data) => {
        console.log('✓ Received connection_status event:', data);
        const statusMsg = data.esp32 ? 'Connected' : 'ESP32 Offline';
        console.log(`  → Updating status to: ${statusMsg} (esp32=${data.esp32})`);
        updateConnectionStatus(data.esp32, statusMsg);
    });

    socket.on('odom_update', (data) => {
        updateOdometryDisplay(data);
    });

    socket.on('pose_update', (data) => {
        updatePoseDisplay(data);
        if (typeof handlePoseUpdate === 'function') {
            handlePoseUpdate(data);
        }
    });

    socket.on('error', (data) => {
        console.error('Server error:', data.message);
        alert(`Error: ${data.message}`);
    });

    socket.on('lidar_scan', (data) => {
        if (typeof handleLidarScan === 'function') {
            handleLidarScan(data);
        }
    });

    socket.on('robot_status', (data) => {
        updateConfigDisplay(data);
    });

    socket.on('config_saved', (data) => {
        if (data.success) {
            showConfigStatus('Configuration saved to EEPROM', 'success');
        }
    });

    socket.on('robot_params', (data) => {
        console.log('Received robot parameters:', data);
        document.getElementById('param-wheel-dia').value = data.wheel_diameter_mm.toFixed(1);
        document.getElementById('param-wheelbase').value = data.wheelbase_mm.toFixed(1);
        document.getElementById('param-ticks-per-rev').value = data.ticks_per_revolution;
        document.getElementById('param-lidar-x').value = data.lidar_offset_x_mm.toFixed(1);
        document.getElementById('param-lidar-y').value = data.lidar_offset_y_mm.toFixed(1);
        updateRobotParamsStatus('Parameters loaded');
    });

    socket.on('status', (data) => {
        console.log('Status:', data.message);
        updateRobotParamsStatus(data.message);
    });

    socket.on('command_ack', (data) => {
        console.log('✓ ACK:', data);
        const msg = data.success ?
            `✓ ${data.message_type} command successful` :
            `✗ ${data.message_type} failed (error ${data.error_code})`;
        showConfigStatus(msg, data.success ? 'success' : 'error');
    });

    socket.on('command_nack', (data) => {
        console.error('✗ NACK:', data);
        const msg = `✗ ${data.message_type} failed: ${data.error_description}`;
        showConfigStatus(msg, 'error');
    });

    // Occupancy grid map updates (Phase 5)
    socket.on('map_update', (data) => {
        if (occupancyMap) {
            occupancyMap.updateMap(data);

            // Update map statistics if UI elements exist
            if (data.stats) {
                const scansElem = document.getElementById('map-scans');
                const updatesElem = document.getElementById('map-updates');
                if (scansElem) scansElem.textContent = data.stats.scans;
                if (updatesElem) updatesElem.textContent = data.stats.updates;
            }
        }
    });
}

/**
 * Initialize UI Controls
 */
function initializeControls() {
    // Direction buttons
    const directionButtons = document.querySelectorAll('.btn-direction');
    directionButtons.forEach(btn => {
        btn.addEventListener('mousedown', () => handleDirectionPress(btn.dataset.direction));
        btn.addEventListener('mouseup', () => handleDirectionRelease());
        btn.addEventListener('mouseleave', () => handleDirectionRelease());

        // Touch events for mobile
        btn.addEventListener('touchstart', (e) => {
            e.preventDefault();
            handleDirectionPress(btn.dataset.direction);
        });
        btn.addEventListener('touchend', (e) => {
            e.preventDefault();
            handleDirectionRelease();
        });
    });

    // Stop button
    const stopButton = document.getElementById('btn-stop');
    stopButton.addEventListener('click', () => {
        console.log('Stop button clicked');
        stopMotor();
    });

    // Velocity slider
    const velocitySlider = document.getElementById('velocity-slider');
    const velocityValue = document.getElementById('velocity-value');

    velocitySlider.addEventListener('input', (e) => {
        currentVelocity = parseFloat(e.target.value);
        velocityValue.textContent = `${currentVelocity.toFixed(2)} m/s`;
    });

    // Streaming toggle button
    const streamButton = document.getElementById('btn-toggle-stream');
    streamButton.addEventListener('click', () => {
        streamingEnabled = !streamingEnabled;
        toggleOdometryStreaming(streamingEnabled);
        streamButton.textContent = `Streaming: ${streamingEnabled ? 'ON' : 'OFF'}`;
        streamButton.classList.toggle('stream-off', !streamingEnabled);
        console.log(`Odometry streaming: ${streamingEnabled ? 'ENABLED' : 'DISABLED'}`);
    });

    // Configuration panel buttons
    const btnReadConfig = document.getElementById('btn-read-config');
    const btnApplyPid = document.getElementById('btn-apply-pid');
    const btnApplyDeadband = document.getElementById('btn-apply-deadband');
    const btnApplyHeadingHold = document.getElementById('btn-apply-heading-hold');
    const btnSaveConfig = document.getElementById('btn-save-config');

    btnReadConfig.addEventListener('click', requestRobotStatus);
    btnApplyPid.addEventListener('click', applyPidSettings);
    btnApplyDeadband.addEventListener('click', applyDeadbandSettings);
    btnApplyHeadingHold.addEventListener('click', applyHeadingHoldKp);
    btnSaveConfig.addEventListener('click', saveConfigToEEPROM);

    // Reset pose button
    const btnResetPose = document.getElementById('btn-reset-pose');
    btnResetPose.addEventListener('click', resetPose);

    // Zero encoders button
    const btnZeroEncoders = document.getElementById('btn-zero-encoders');
    btnZeroEncoders.addEventListener('click', zeroEncoders);

    // Robot parameters buttons
    const btnGetRobotParams = document.getElementById('btn-get-robot-params');
    const btnApplyRobotParams = document.getElementById('btn-apply-robot-params');
    const btnSaveRobotParams = document.getElementById('btn-save-robot-params');

    btnGetRobotParams.addEventListener('click', getRobotParameters);
    btnApplyRobotParams.addEventListener('click', applyRobotParameters);
    btnSaveRobotParams.addEventListener('click', saveRobotParameters);

    // Map control buttons (Phase 5)
    const btnClearMap = document.getElementById('btn-clear-map');
    if (btnClearMap) {
        btnClearMap.addEventListener('click', clearOccupancyMap);
    }

    // Map origin configuration (Phase 5)
    const presetButtons = document.querySelectorAll('.btn-preset');
    presetButtons.forEach(btn => {
        btn.addEventListener('click', () => handleOriginPreset(btn.dataset.preset));
    });

    const btnApplyOrigin = document.getElementById('btn-apply-origin');
    if (btnApplyOrigin) {
        btnApplyOrigin.addEventListener('click', applyMapOrigin);
    }

    // Request initial status on startup
    setTimeout(() => {
        if (isConnected) {
            requestRobotStatus();
            getRobotParameters();  // Also get robot parameters
        }
    }, 1000);
}

/**
 * Keyboard Controls
 */
function initializeKeyboardControls() {
    document.addEventListener('keydown', (e) => {
        // Prevent keyboard control if user is typing in an input
        if (e.target.tagName === 'INPUT') return;

        // Handle SPACE key (stop)
        if (e.key === ' ') {
            e.preventDefault();
            stopMotor();
            return;
        }

        // Map keys to directions
        let direction = null;
        const key = e.key.toLowerCase();
        switch(key) {
            case 'w':
            case 'arrowup':
                direction = 'forward';
                break;
            case 'x':
            case 'arrowdown':
                direction = 'backward';
                break;
            case 'a':
            case 'arrowleft':
                direction = 'left';
                break;
            case 'd':
            case 'arrowright':
                direction = 'right';
                break;
            case 's':
                e.preventDefault();
                stopMotor();
                return;
        }

        // Only trigger if this is a NEW key press (not auto-repeat)
        if (direction && !pressedKeys.has(key)) {
            e.preventDefault();
            pressedKeys.add(key);
            handleDirectionPress(direction);
        }
    });

    document.addEventListener('keyup', (e) => {
        if (e.target.tagName === 'INPUT') return;

        const key = e.key.toLowerCase();
        if (['w', 'x', 'a', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(key)) {
            e.preventDefault();
            pressedKeys.delete(key);
            handleDirectionRelease();
        }
    });
}

/**
 * Motor Control Functions
 */
function handleDirectionPress(direction) {
    console.log(`Direction pressed: ${direction}`);
    activeDirection = direction;

    // Visual feedback
    highlightDirectionButton(direction);

    // Send motor command ONCE - no interval
    sendMotorCommand(direction);
}

function handleDirectionRelease() {
    if (!activeDirection) return;

    console.log(`Direction released: ${activeDirection}`);

    // Remove visual feedback only - motors continue at set velocity
    removeDirectionHighlight();

    // Clear active direction
    activeDirection = null;
}

function sendMotorCommand(direction) {
    if (!socket || !isConnected) {
        console.warn('Not connected to server');
        return;
    }

    // For forward/backward: use cmd_vel with w=0 (enables heading hold)
    // For left/right: use direct wheel velocities (legacy mode)

    if (direction === 'forward' || direction === 'backward') {
        // Use cmd_vel mode: v (linear velocity), w=0 (heading hold)
        const v = (direction === 'forward') ? currentVelocity : -currentVelocity;

        socket.emit('motor_command', {
            type: 'cmd_vel',
            v: v,
            w: 0.0  // w=0 activates heading hold with angular velocity feedback
        });

        console.log(`Motor command (cmd_vel): v=${v.toFixed(2)} m/s, w=0 rad/s (heading hold)`);

    } else {
        // For turning: use direct wheel velocities (legacy mode)
        let vel_left = 0;
        let vel_right = 0;

        switch(direction) {
            case 'left':
                vel_left = -currentVelocity * 0.5;
                vel_right = currentVelocity * 0.5;
                break;
            case 'right':
                vel_left = currentVelocity * 0.5;
                vel_right = -currentVelocity * 0.5;
                break;
        }

        socket.emit('motor_command', {
            type: 'velocity',
            vel_left: vel_left,
            vel_right: vel_right
        });

        console.log(`Motor command (wheel vel): L=${vel_left.toFixed(2)} R=${vel_right.toFixed(2)} m/s`);
    }
}

function stopMotor() {
    if (!socket || !isConnected) {
        console.warn('stopMotor: Not connected to server');
        return;
    }

    console.log('🛑 Motor STOP called');

    // Clear state
    activeDirection = null;
    pressedKeys.clear();

    // Remove visual feedback
    removeDirectionHighlight();

    // Send STOP command
    console.log('  → Sending STOP command...');
    socket.emit('motor_command', {
        type: 'stop'
    });

    console.log('✓ Motor STOP command sent');
}

/**
 * Odometry Streaming Control
 */
function toggleOdometryStreaming(enable) {
    if (!socket || !isConnected) {
        console.warn('toggleOdometryStreaming: Not connected to server');
        return;
    }

    console.log(`${enable ? 'Enabling' : 'Disabling'} odometry streaming...`);
    socket.emit('enable_stream', {
        enable: enable,
        interval_ms: 200  // 5 Hz
    });
    console.log(`✓ Streaming ${enable ? 'enabled' : 'disabled'} @ 200ms`);
}

/**
 * UI Update Functions
 */
function updateConnectionStatus(connected, message) {
    const statusElement = document.getElementById('esp32-status');

    if (connected) {
        statusElement.className = 'status-online';
        statusElement.textContent = `ESP32: ${message}`;
    } else {
        statusElement.className = 'status-offline';
        statusElement.textContent = `ESP32: ${message}`;
    }
}

function updateOdometryDisplay(data) {
    // Update velocity (m/s)
    document.getElementById('vel-left').textContent = `${data.vel_left.toFixed(2)} m/s`;
    document.getElementById('vel-right').textContent = `${data.vel_right.toFixed(2)} m/s`;

    // Calculate and update velocity (ticks/s)
    // Assuming data contains velocityTicksLeft and velocityTicksRight from server
    // If not, we can calculate from vel_left/right and wheel radius
    const velLeftTicks = data.velocity_ticks_left || Math.round(data.vel_left * 360 / (0.082 * Math.PI));
    const velRightTicks = data.velocity_ticks_right || Math.round(data.vel_right * 360 / (0.082 * Math.PI));
    document.getElementById('vel-left-ticks').textContent = velLeftTicks;
    document.getElementById('vel-right-ticks').textContent = velRightTicks;

    // Update encoders
    document.getElementById('enc-left').textContent = data.encoder_left;
    document.getElementById('enc-right').textContent = data.encoder_right;

    // Update PWM
    document.getElementById('pwm-left').textContent = data.pwm_left;
    document.getElementById('pwm-right').textContent = data.pwm_right;
}

function highlightDirectionButton(direction) {
    // Remove previous highlights
    removeDirectionHighlight();

    // Add highlight to active button
    const button = document.querySelector(`.btn-direction[data-direction="${direction}"]`);
    if (button) {
        button.classList.add('active');
    }
}

function removeDirectionHighlight() {
    const buttons = document.querySelectorAll('.btn-direction');
    buttons.forEach(btn => btn.classList.remove('active'));
}

function updatePoseDisplay(data) {
    // Host odometry (dead reckoning) pose
    if (data.dead_reckoning) {
        const dr = data.dead_reckoning;
        document.getElementById('pose-dr-x').textContent = `${dr.x.toFixed(3)} m`;
        document.getElementById('pose-dr-y').textContent = `${dr.y.toFixed(3)} m`;
        document.getElementById('pose-dr-theta').textContent = `${dr.theta_deg.toFixed(1)}°`;
    }

    // Arduino odometry pose
    if (data.arduino_odo) {
        const ard = data.arduino_odo;
        document.getElementById('pose-ard-x').textContent = `${ard.x.toFixed(3)} m`;
        document.getElementById('pose-ard-y').textContent = `${ard.y.toFixed(3)} m`;
        document.getElementById('pose-ard-theta').textContent = `${ard.theta_deg.toFixed(1)}°`;
    }

    // ICP-corrected pose
    if (data.icp_corrected) {
        const icp = data.icp_corrected;
        document.getElementById('pose-icp-x').textContent = `${icp.x.toFixed(3)} m`;
        document.getElementById('pose-icp-y').textContent = `${icp.y.toFixed(3)} m`;
        document.getElementById('pose-icp-theta').textContent = `${icp.theta_deg.toFixed(1)}°`;
    }

    // Drift
    if (data.drift) {
        document.getElementById('pose-drift-pos').textContent = `${data.drift.position.toFixed(3)} m`;
        document.getElementById('pose-drift-theta').textContent = `${data.drift.heading.toFixed(1)}°`;
    }
}

/**
 * Configuration Panel Functions
 */
function requestRobotStatus() {
    if (!socket || !isConnected) {
        showConfigStatus('Not connected to server', 'error');
        return;
    }

    console.log('Requesting robot status...');
    socket.emit('request_status');
    showConfigStatus('Reading from Arduino...', '');
}

function updateConfigDisplay(data) {
    // Update per-motor PID values (new UI)
    if (data.pid_left && data.pid_right) {
        // Per-motor PID values
        document.getElementById('pid-left-kp').value = data.pid_left.kp.toFixed(2);
        document.getElementById('pid-left-ki').value = data.pid_left.ki.toFixed(2);
        document.getElementById('pid-left-kd').value = data.pid_left.kd.toFixed(2);
        document.getElementById('pid-right-kp').value = data.pid_right.kp.toFixed(2);
        document.getElementById('pid-right-ki').value = data.pid_right.ki.toFixed(2);
        document.getElementById('pid-right-kd').value = data.pid_right.kd.toFixed(2);
    } else if (data.pid) {
        // Legacy: same PID for both motors
        document.getElementById('pid-left-kp').value = data.pid.kp.toFixed(2);
        document.getElementById('pid-left-ki').value = data.pid.ki.toFixed(2);
        document.getElementById('pid-left-kd').value = data.pid.kd.toFixed(2);
        document.getElementById('pid-right-kp').value = data.pid.kp.toFixed(2);
        document.getElementById('pid-right-ki').value = data.pid.ki.toFixed(2);
        document.getElementById('pid-right-kd').value = data.pid.kd.toFixed(2);
    }

    // Update deadband values
    if (data.deadband) {
        document.getElementById('db-left-fwd').value = data.deadband.left_forward.toFixed(1);
        document.getElementById('db-left-rev').value = data.deadband.left_reverse.toFixed(1);
        document.getElementById('db-right-fwd').value = data.deadband.right_forward.toFixed(1);
        document.getElementById('db-right-rev').value = data.deadband.right_reverse.toFixed(1);
    }

    // Update heading hold Kp
    if (data.heading_hold_kp !== undefined) {
        document.getElementById('heading-hold-kp').value = data.heading_hold_kp.toFixed(1);
    }

    console.log('Updated config display:', data);
    showConfigStatus('Configuration loaded', 'success');
}

function applyPidSettings() {
    if (!socket || !isConnected) {
        showConfigStatus('Not connected to server', 'error');
        return;
    }

    const leftKp = parseFloat(document.getElementById('pid-left-kp').value);
    const leftKi = parseFloat(document.getElementById('pid-left-ki').value);
    const leftKd = parseFloat(document.getElementById('pid-left-kd').value);
    const rightKp = parseFloat(document.getElementById('pid-right-kp').value);
    const rightKi = parseFloat(document.getElementById('pid-right-ki').value);
    const rightKd = parseFloat(document.getElementById('pid-right-kd').value);

    console.log(`Applying per-motor PID: Left(${leftKp},${leftKi},${leftKd}) Right(${rightKp},${rightKi},${rightKd})`);
    socket.emit('set_pid_per_motor', { leftKp, leftKi, leftKd, rightKp, rightKi, rightKd });
    showConfigStatus('Applying per-motor PID settings...', '');
}

function applyDeadbandSettings() {
    if (!socket || !isConnected) {
        showConfigStatus('Not connected to server', 'error');
        return;
    }

    const leftFwd = parseFloat(document.getElementById('db-left-fwd').value);
    const leftRev = parseFloat(document.getElementById('db-left-rev').value);
    const rightFwd = parseFloat(document.getElementById('db-right-fwd').value);
    const rightRev = parseFloat(document.getElementById('db-right-rev').value);

    console.log(`Applying Deadband: L_fwd=${leftFwd}, L_rev=${leftRev}, R_fwd=${rightFwd}, R_rev=${rightRev}`);
    socket.emit('set_deadband', {
        left_forward: leftFwd,
        left_reverse: leftRev,
        right_forward: rightFwd,
        right_reverse: rightRev
    });
    showConfigStatus('Applying deadband settings...', '');
}

function applyHeadingHoldKp() {
    if (!socket || !isConnected) {
        showConfigStatus('Not connected to server', 'error');
        return;
    }

    const kp = parseFloat(document.getElementById('heading-hold-kp').value);

    console.log(`Applying Heading Hold Kp: ${kp}`);
    socket.emit('set_heading_hold_kp', { kp: kp });
    showConfigStatus('Applying heading hold Kp...', '');
}

function saveConfigToEEPROM() {
    if (!socket || !isConnected) {
        showConfigStatus('Not connected to server', 'error');
        return;
    }

    console.log('Saving configuration to EEPROM...');
    socket.emit('save_config');
    showConfigStatus('Saving to EEPROM...', '');
}

function showConfigStatus(message, type) {
    const statusElement = document.getElementById('config-status');
    statusElement.textContent = message;
    statusElement.className = 'config-status';
    if (type) {
        statusElement.classList.add(type);
    }

    // Clear status after 3 seconds
    if (type === 'success' || type === 'error') {
        setTimeout(() => {
            statusElement.textContent = 'Ready';
            statusElement.className = 'config-status';
        }, 3000);
    }
}

/**
 * Pose Reset Function
 */
function resetPose() {
    if (!socket || !isConnected) {
        alert('Not connected to server');
        return;
    }

    if (confirm('Reset robot pose to origin (0, 0, 0°)?')) {
        console.log('Resetting pose to origin...');
        socket.emit('reset_pose');
    }
}

/**
 * Zero encoder counts and reset pose
 */
function zeroEncoders() {
    if (!socket || !isConnected) {
        alert('Not connected to server');
        return;
    }

    if (confirm('Zero Arduino encoders and reset all odometry to origin?')) {
        console.log('Zeroing encoders and resetting pose...');
        socket.emit('zero_encoders');
    }
}

/**
 * Clear occupancy grid map (Phase 5)
 */
function clearOccupancyMap() {
    if (!socket || !isConnected) {
        alert('Not connected to server');
        return;
    }

    if (confirm('Clear the occupancy grid map? This will reset all mapping data.')) {
        console.log('Clearing occupancy map...');
        socket.emit('clear_map');
    }
}

/**
 * Handle map origin preset selection (Phase 5)
 */
function handleOriginPreset(preset) {
    const inputsDiv = document.getElementById('origin-inputs');
    const xInput = document.getElementById('origin-offset-x');
    const yInput = document.getElementById('origin-offset-y');
    const coverageText = document.getElementById('origin-coverage');

    // Update button active state
    document.querySelectorAll('.btn-preset').forEach(btn => {
        btn.classList.remove('btn-preset-active');
    });
    event.target.classList.add('btn-preset-active');

    switch(preset) {
        case 'centered':
            xInput.value = 0.0;
            yInput.value = 0.0;
            inputsDiv.style.display = 'none';
            updateCoverageText(0.0, 0.0);
            applyMapOrigin();
            break;
        case 'corner':
            xInput.value = -2.5;
            yInput.value = -2.5;
            inputsDiv.style.display = 'none';
            updateCoverageText(-2.5, -2.5);
            applyMapOrigin();
            break;
        case 'custom':
            inputsDiv.style.display = 'flex';
            break;
    }
}

/**
 * Apply map origin offset (Phase 5)
 */
function applyMapOrigin() {
    if (!socket || !isConnected) {
        alert('Not connected to server');
        return;
    }

    const xOffset = parseFloat(document.getElementById('origin-offset-x').value);
    const yOffset = parseFloat(document.getElementById('origin-offset-y').value);

    console.log(`Setting map origin offset to (${xOffset}, ${yOffset})`);
    socket.emit('set_map_origin', {
        origin_offset_x: xOffset,
        origin_offset_y: yOffset
    });

    updateCoverageText(xOffset, yOffset);
}

/**
 * Update coverage text display (Phase 5)
 */
function updateCoverageText(xOffset, yOffset) {
    const gridSize = 7.0; // 7m × 7m grid
    const halfGrid = gridSize / 2;

    const xMin = xOffset - halfGrid;
    const xMax = xOffset + halfGrid;
    const yMin = yOffset - halfGrid;
    const yMax = yOffset + halfGrid;

    const coverageText = document.getElementById('origin-coverage');
    coverageText.textContent = `Coverage: X: ${xMin.toFixed(1)}m to ${xMax.toFixed(1)}m, Y: ${yMin.toFixed(1)}m to ${yMax.toFixed(1)}m`;
}

/**
 * Robot Parameters Functions
 */
function getRobotParameters() {
    if (!socket || !isConnected) {
        alert('Not connected to server');
        return;
    }

    console.log('Requesting robot parameters...');
    socket.emit('get_robot_params');
    updateRobotParamsStatus('Reading parameters...');
}

function applyRobotParameters() {
    if (!socket || !isConnected) {
        alert('Not connected to server');
        return;
    }

    const params = {
        wheel_diameter_mm: parseFloat(document.getElementById('param-wheel-dia').value),
        wheelbase_mm: parseFloat(document.getElementById('param-wheelbase').value),
        ticks_per_revolution: parseInt(document.getElementById('param-ticks-per-rev').value),
        lidar_offset_x_mm: parseFloat(document.getElementById('param-lidar-x').value),
        lidar_offset_y_mm: parseFloat(document.getElementById('param-lidar-y').value),
    };

    console.log('Applying robot parameters:', params);
    socket.emit('set_robot_params', params);
    updateRobotParamsStatus('Applying parameters...');
}

function saveRobotParameters() {
    if (!socket || !isConnected) {
        alert('Not connected to server');
        return;
    }

    if (confirm('Save robot parameters to file?')) {
        console.log('Saving robot parameters...');
        socket.emit('save_robot_params');
        updateRobotParamsStatus('Saving to file...');
    }
}

function updateRobotParamsStatus(message) {
    const statusEl = document.getElementById('robot-params-status');
    if (statusEl) {
        statusEl.textContent = message;
        statusEl.style.color = '#4fc3f7';

        // Reset to 'Ready' after 3 seconds
        setTimeout(() => {
            statusEl.textContent = 'Ready';
            statusEl.style.color = '';
        }, 3000);
    }
}

/**
 * Utility Functions
 */
function log(message, type = 'info') {
    const timestamp = new Date().toLocaleTimeString();
    console.log(`[${timestamp}] ${type.toUpperCase()}: ${message}`);
}

// Prevent accidental page navigation
window.addEventListener('beforeunload', (e) => {
    if (activeDirection) {
        stopMotor();
    }
});
