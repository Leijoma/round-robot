/**
 * LIDAR Visualization - Polar Plot
 * Real-time 360° LIDAR data visualization using HTML5 Canvas
 */

// LIDAR state
let lidarEnabled = false;
let lidarCanvas = null;
let lidarCtx = null;
let lidarData = new Map(); // Store readings by angle for persistence
let lastLidarUpdate = 0;
let lidarRPM = 0;

/**
 * Initialize LIDAR visualization
 */
function initializeLidarVisualization() {
    console.log('Initializing LIDAR visualization...');

    // Get canvas and context
    lidarCanvas = document.getElementById('lidar-canvas');
    if (!lidarCanvas) {
        console.error('LIDAR canvas not found!');
        return;
    }
    lidarCtx = lidarCanvas.getContext('2d');

    // Initialize controls
    const btnLidarEnable = document.getElementById('btn-lidar-enable');
    const btnSetRpm = document.getElementById('btn-set-rpm');
    const rpmInput = document.getElementById('lidar-rpm');

    btnLidarEnable.addEventListener('click', () => {
        lidarEnabled = !lidarEnabled;
        toggleLidar(lidarEnabled);
        btnLidarEnable.textContent = `LIDAR: ${lidarEnabled ? 'ON' : 'OFF'}`;
        btnLidarEnable.classList.toggle('active', lidarEnabled);
    });

    btnSetRpm.addEventListener('click', () => {
        const rpm = parseInt(rpmInput.value);
        if (rpm >= 200 && rpm <= 300) {
            setLidarRPM(rpm);
        } else {
            alert('RPM must be between 200 and 300');
        }
    });


    // Start rendering loop
    renderLidarPlot();
    console.log('LIDAR visualization initialized');
}

/**
 * Toggle LIDAR motor on/off
 */
function toggleLidar(enable) {
    if (!socket || !isConnected) {
        console.warn('Cannot toggle LIDAR: Not connected');
        return;
    }

    console.log(`${enable ? 'Enabling' : 'Disabling'} LIDAR...`);
    socket.emit('lidar_enable', { enable: enable });

    if (!enable) {
        // Clear data when disabled
        lidarData.clear();
    }
}

/**
 * Set LIDAR target RPM
 */
function setLidarRPM(rpm) {
    if (!socket || !isConnected) {
        console.warn('Cannot set LIDAR RPM: Not connected');
        return;
    }

    console.log(`Setting LIDAR RPM to ${rpm}`);
    socket.emit('lidar_set_rpm', { rpm: rpm });
}

/**
 * Handle incoming LIDAR scan data
 */
function handleLidarScan(data) {
    lastLidarUpdate = Date.now();
    lidarRPM = data.rpm;

    // Update lidarData map with new readings
    for (const reading of data.readings) {
        if (reading.valid) {
            lidarData.set(reading.angle, {
                distance: reading.distance,
                strength: reading.strength,
                timestamp: lastLidarUpdate
            });
        }
    }

    // Remove old readings (older than 2 seconds)
    const cutoffTime = lastLidarUpdate - 2000;
    for (const [angle, reading] of lidarData) {
        if (reading.timestamp < cutoffTime) {
            lidarData.delete(angle);
        }
    }

    // Update status display
    updateLidarStatus();
}

/**
 * Update LIDAR status display
 */
function updateLidarStatus() {
    const statusElement = document.getElementById('lidar-status');
    if (!statusElement) return;

    if (lidarData.size === 0) {
        statusElement.textContent = 'No data';
        statusElement.className = 'status-text status-inactive';
    } else {
        const age = Date.now() - lastLidarUpdate;
        const ageText = age < 1000 ? 'Live' : `${(age / 1000).toFixed(1)}s ago`;
        statusElement.textContent = `${lidarData.size} points | ${lidarRPM} RPM | ${ageText}`;
        statusElement.className = 'status-text status-active';
    }
}

/**
 * Render polar plot (runs continuously via requestAnimationFrame)
 */
function renderLidarPlot() {
    if (!lidarCtx || !lidarCanvas) return;

    const ctx = lidarCtx;
    const width = lidarCanvas.width;
    const height = lidarCanvas.height;
    const centerX = width / 2;
    const centerY = height / 2;
    const maxRadius = Math.min(centerX, centerY) - 20;

    // Safety check: prevent rendering if canvas has invalid dimensions
    if (maxRadius <= 0 || !isFinite(maxRadius)) {
        requestAnimationFrame(renderLidarPlot);
        return;
    }

    // Clear canvas
    ctx.fillStyle = '#1a1a1a';
    ctx.fillRect(0, 0, width, height);

    // Draw grid circles (distance rings)
    ctx.strokeStyle = '#333';
    ctx.lineWidth = 1;
    for (let r = maxRadius; r > 0; r -= maxRadius / 5) {
        ctx.beginPath();
        ctx.arc(centerX, centerY, r, 0, 2 * Math.PI);
        ctx.stroke();
    }

    // Draw grid lines (angle lines)
    ctx.strokeStyle = '#333';
    for (let angle = 0; angle < 360; angle += 30) {
        const rad = (angle - 90) * Math.PI / 180;
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.lineTo(
            centerX + maxRadius * Math.cos(rad),
            centerY + maxRadius * Math.sin(rad)
        );
        ctx.stroke();
    }

    // Draw distance labels
    ctx.fillStyle = '#666';
    ctx.font = '10px monospace';
    ctx.textAlign = 'center';
    for (let i = 1; i <= 5; i++) {
        const r = (maxRadius / 5) * i;
        const dist = (i * 500).toFixed(0); // Max range 2.5m (5 rings * 500mm)
        ctx.fillText(`${dist}mm`, centerX, centerY - r + 3);
    }

    // Draw cardinal directions
    ctx.fillStyle = '#888';
    ctx.font = '12px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('0°', centerX, centerY - maxRadius - 10);
    ctx.fillText('180°', centerX, centerY + maxRadius + 10);
    ctx.textAlign = 'left';
    ctx.fillText('90°', centerX + maxRadius + 5, centerY);
    ctx.textAlign = 'right';
    ctx.fillText('270°', centerX - maxRadius - 5, centerY);

    // Draw LIDAR points
    if (lidarData.size > 0) {
        // Scale factor: map distance in mm to pixels
        const maxDistance = 2500; // 2.5 meters max range
        const scale = maxRadius / maxDistance;

        for (const [angle, reading] of lidarData) {
            const distance = reading.distance;
            const strength = reading.strength;

            // Convert to canvas coordinates
            // LIDAR angle 0° = forward (canvas 270°)
            const angleRad = ((angle - 90) * Math.PI) / 180;
            const r = distance * scale;

            // Negate X to fix left/right mirroring
            const x = centerX - r * Math.cos(angleRad);
            const y = centerY + r * Math.sin(angleRad);

            // All points in green
            // Size based on signal strength
            const pointSize = 2 + (strength / 16384) * 2; // Assuming max strength ~16384

            ctx.fillStyle = '#00ff00';  // Green
            ctx.beginPath();
            ctx.arc(x, y, pointSize, 0, 2 * Math.PI);
            ctx.fill();
        }
    }

    // Draw robot position (center)
    ctx.fillStyle = '#00ff00';
    ctx.beginPath();
    ctx.arc(centerX, centerY, 4, 0, 2 * Math.PI);
    ctx.fill();

    // Draw robot heading indicator (forward direction)
    ctx.strokeStyle = '#00ff00';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(centerX, centerY - 15);
    ctx.stroke();

    // Continue rendering loop
    requestAnimationFrame(renderLidarPlot);
}

// Update status periodically
setInterval(updateLidarStatus, 500);

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    initializeLidarVisualization();
});
