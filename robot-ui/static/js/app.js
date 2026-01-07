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

// Initialize application
document.addEventListener('DOMContentLoaded', () => {
    console.log('Robot Control UI initializing...');
    initializeWebSocket();
    initializeControls();
    initializeKeyboardControls();
});

/**
 * WebSocket Connection
 */
function initializeWebSocket() {
    // Connect to Flask-SocketIO server
    socket = io();

    socket.on('connect', () => {
        console.log('✓ Connected to server');
        isConnected = true;
        updateConnectionStatus(true, 'Connecting to ESP32...');
    });

    socket.on('disconnect', () => {
        console.log('✗ Disconnected from server');
        isConnected = false;
        updateConnectionStatus(false, 'Disconnected');
        stopMotor();
    });

    socket.on('connection_status', (data) => {
        console.log('Connection status:', data);
        updateConnectionStatus(data.esp32, data.esp32 ? 'Connected' : 'ESP32 Offline');
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
    const btnSaveConfig = document.getElementById('btn-save-config');

    btnReadConfig.addEventListener('click', requestRobotStatus);
    btnApplyPid.addEventListener('click', applyPidSettings);
    btnApplyDeadband.addEventListener('click', applyDeadbandSettings);
    btnSaveConfig.addEventListener('click', saveConfigToEEPROM);

    // Reset pose button
    const btnResetPose = document.getElementById('btn-reset-pose');
    btnResetPose.addEventListener('click', resetPose);

    // Request initial status on startup
    setTimeout(() => {
        if (isConnected) {
            requestRobotStatus();
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
            case 's':
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
        if (['w', 's', 'a', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(key)) {
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

    let vel_left = 0;
    let vel_right = 0;

    // Calculate wheel velocities based on direction
    switch(direction) {
        case 'forward':
            vel_left = currentVelocity;
            vel_right = currentVelocity;
            break;
        case 'backward':
            vel_left = -currentVelocity;
            vel_right = -currentVelocity;
            break;
        case 'left':
            vel_left = -currentVelocity * 0.5;
            vel_right = currentVelocity * 0.5;
            break;
        case 'right':
            vel_left = currentVelocity * 0.5;
            vel_right = -currentVelocity * 0.5;
            break;
    }

    // Send command via WebSocket
    socket.emit('motor_command', {
        type: 'velocity',
        vel_left: vel_left,
        vel_right: vel_right
    });

    console.log(`Motor command: L=${vel_left.toFixed(2)} R=${vel_right.toFixed(2)} m/s`);
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
    // Dead reckoning pose
    if (data.dead_reckoning) {
        const dr = data.dead_reckoning;
        document.getElementById('pose-dr-x').textContent = `${dr.x.toFixed(3)} m`;
        document.getElementById('pose-dr-y').textContent = `${dr.y.toFixed(3)} m`;
        document.getElementById('pose-dr-theta').textContent = `${dr.theta_deg.toFixed(1)}°`;
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
    // Update PID values
    if (data.pid) {
        document.getElementById('pid-kp').value = data.pid.kp.toFixed(2);
        document.getElementById('pid-ki').value = data.pid.ki.toFixed(2);
        document.getElementById('pid-kd').value = data.pid.kd.toFixed(2);
    }

    // Update deadband values
    if (data.deadband) {
        document.getElementById('db-left-fwd').value = data.deadband.left_forward.toFixed(1);
        document.getElementById('db-left-rev').value = data.deadband.left_reverse.toFixed(1);
        document.getElementById('db-right-fwd').value = data.deadband.right_forward.toFixed(1);
        document.getElementById('db-right-rev').value = data.deadband.right_reverse.toFixed(1);
    }

    console.log('Updated config display:', data);
    showConfigStatus('Configuration loaded', 'success');
}

function applyPidSettings() {
    if (!socket || !isConnected) {
        showConfigStatus('Not connected to server', 'error');
        return;
    }

    const kp = parseFloat(document.getElementById('pid-kp').value);
    const ki = parseFloat(document.getElementById('pid-ki').value);
    const kd = parseFloat(document.getElementById('pid-kd').value);

    console.log(`Applying PID: Kp=${kp}, Ki=${ki}, Kd=${kd}`);
    socket.emit('set_pid', { kp, ki, kd });
    showConfigStatus('Applying PID settings...', '');
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
