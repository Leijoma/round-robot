/**
 * Map Canvas - Robot Path Visualization
 * Shows robot position and traveled path relative to start position
 * Displays three odometry sources: Host (orange), Arduino (blue), ICP-corrected (green)
 */

// Map state
let mapCanvas = null;
let mapCtx = null;
let pathHistory = {
    dead_reckoning: [],  // Array of {x, y, theta} poses - Host odometry (orange)
    arduino_odo: [],     // Array of {x, y, theta} poses - Arduino odometry (blue)
    icp_corrected: []    // Array of {x, y, theta} poses - ICP corrected (green)
};
const MAX_HISTORY = 1000;
let lastMapUpdate = 0;

// View parameters
let viewScale = 100;  // pixels per meter (will auto-scale)
let viewOffsetX = 0;  // pan offset in pixels
let viewOffsetY = 0;

// Visibility toggles
let showOdomPath = true;     // Host odometry (orange)
let showArduinoPath = true;  // Arduino odometry (blue)
let showIcpPath = true;      // ICP corrected (green)

/**
 * Initialize map visualization
 */
function initializeMapVisualization() {
    console.log('Initializing map visualization...');

    // Get canvas and context
    mapCanvas = document.getElementById('map-canvas');
    if (!mapCanvas) {
        console.error('Map canvas not found!');
        return;
    }
    mapCtx = mapCanvas.getContext('2d');

    // Initialize clear button
    const btnClearPath = document.getElementById('btn-clear-path');
    btnClearPath.addEventListener('click', () => {
        clearPath();
    });

    // Initialize toggle buttons
    const btnToggleOdom = document.getElementById('btn-toggle-odom');
    const btnToggleArduino = document.getElementById('btn-toggle-arduino');
    const btnToggleIcp = document.getElementById('btn-toggle-icp');

    btnToggleOdom.addEventListener('click', () => {
        showOdomPath = !showOdomPath;
        if (showOdomPath) {
            btnToggleOdom.classList.remove('btn-toggle-inactive');
            btnToggleOdom.classList.add('btn-toggle-active');
            btnToggleOdom.textContent = 'Show Host';
        } else {
            btnToggleOdom.classList.remove('btn-toggle-active');
            btnToggleOdom.classList.add('btn-toggle-inactive');
            btnToggleOdom.textContent = 'Hide Host';
        }
    });

    btnToggleArduino.addEventListener('click', () => {
        showArduinoPath = !showArduinoPath;
        if (showArduinoPath) {
            btnToggleArduino.classList.remove('btn-toggle-inactive');
            btnToggleArduino.classList.add('btn-toggle-active');
            btnToggleArduino.textContent = 'Show Arduino';
        } else {
            btnToggleArduino.classList.remove('btn-toggle-active');
            btnToggleArduino.classList.add('btn-toggle-inactive');
            btnToggleArduino.textContent = 'Hide Arduino';
        }
    });

    btnToggleIcp.addEventListener('click', () => {
        showIcpPath = !showIcpPath;
        if (showIcpPath) {
            btnToggleIcp.classList.remove('btn-toggle-inactive');
            btnToggleIcp.classList.add('btn-toggle-active');
            btnToggleIcp.textContent = 'Show ICP';
        } else {
            btnToggleIcp.classList.remove('btn-toggle-active');
            btnToggleIcp.classList.add('btn-toggle-inactive');
            btnToggleIcp.textContent = 'Hide ICP';
        }
    });

    // Start rendering loop
    renderMapCanvas();
    console.log('Map visualization initialized');
}

/**
 * Handle incoming pose update (called from app.js)
 */
function handlePoseUpdate(data) {
    lastMapUpdate = Date.now();

    // Add dead reckoning pose to history
    if (data.dead_reckoning) {
        pathHistory.dead_reckoning.push({
            x: data.dead_reckoning.x,
            y: data.dead_reckoning.y,
            theta: data.dead_reckoning.theta_deg * Math.PI / 180  // Convert to radians
        });

        // Limit history size
        if (pathHistory.dead_reckoning.length > MAX_HISTORY) {
            pathHistory.dead_reckoning.shift();
        }
    }

    // Add Arduino odometry pose to history
    if (data.arduino_odo) {
        pathHistory.arduino_odo.push({
            x: data.arduino_odo.x,
            y: data.arduino_odo.y,
            theta: data.arduino_odo.theta_deg * Math.PI / 180
        });

        // Limit history size
        if (pathHistory.arduino_odo.length > MAX_HISTORY) {
            pathHistory.arduino_odo.shift();
        }
    }

    // Add ICP-corrected pose to history
    if (data.icp_corrected) {
        pathHistory.icp_corrected.push({
            x: data.icp_corrected.x,
            y: data.icp_corrected.y,
            theta: data.icp_corrected.theta_deg * Math.PI / 180
        });

        // Limit history size
        if (pathHistory.icp_corrected.length > MAX_HISTORY) {
            pathHistory.icp_corrected.shift();
        }
    }

    // Update status
    updateMapStatus();
}

/**
 * Clear path history
 */
function clearPath() {
    pathHistory.dead_reckoning = [];
    pathHistory.arduino_odo = [];
    pathHistory.icp_corrected = [];
    console.log('Path history cleared');
    updateMapStatus();
}

/**
 * Update map status display
 */
function updateMapStatus() {
    const statusElement = document.getElementById('map-status');
    if (!statusElement) return;

    const drCount = pathHistory.dead_reckoning.length;
    const ardCount = pathHistory.arduino_odo.length;
    const icpCount = pathHistory.icp_corrected.length;

    if (drCount === 0 && ardCount === 0 && icpCount === 0) {
        statusElement.textContent = 'No data';
        statusElement.className = 'status-text status-inactive';
    } else {
        const age = Date.now() - lastMapUpdate;
        const ageText = age < 1000 ? 'Live' : `${(age / 1000).toFixed(1)}s ago`;
        const maxCount = Math.max(drCount, ardCount, icpCount);
        statusElement.textContent = `${maxCount} poses | ${ageText}`;
        statusElement.className = 'status-text status-active';
    }
}

// Throttle auto-scale calculation (only recalculate every 500ms)
let lastAutoScaleUpdate = 0;
const AUTO_SCALE_INTERVAL = 500; // ms

/**
 * Render map canvas (runs continuously via requestAnimationFrame)
 */
function renderMapCanvas() {
    if (!mapCtx || !mapCanvas) {
        requestAnimationFrame(renderMapCanvas);
        return;
    }

    const ctx = mapCtx;
    const width = mapCanvas.width;
    const height = mapCanvas.height;

    // Safety check: ensure canvas has valid dimensions
    if (width <= 0 || height <= 0 || !isFinite(width) || !isFinite(height)) {
        requestAnimationFrame(renderMapCanvas);
        return;
    }

    const centerX = width / 2;
    const centerY = height / 2;

    // Clear canvas
    ctx.fillStyle = '#1a1a1a';
    ctx.fillRect(0, 0, width, height);

    // Calculate auto-scale based on traveled area (throttled)
    const now = Date.now();
    if (now - lastAutoScaleUpdate > AUTO_SCALE_INTERVAL) {
        calculateAutoScale();
        lastAutoScaleUpdate = now;
    }

    // Draw grid
    drawGrid(ctx, width, height, centerX, centerY);

    // Draw paths in order: ICP (background), Host, Arduino (foreground)
    if (showIcpPath) {
        drawPath(ctx, pathHistory.icp_corrected, '#00ff00', centerX, centerY);   // Green
    }
    if (showOdomPath) {
        drawPath(ctx, pathHistory.dead_reckoning, '#ff9500', centerX, centerY);  // Orange
    }
    if (showArduinoPath) {
        drawPath(ctx, pathHistory.arduino_odo, '#00d4ff', centerX, centerY);     // Cyan/Blue
    }

    // Draw three robot triangles at current positions
    if (showIcpPath && pathHistory.icp_corrected.length > 0) {
        const pose = pathHistory.icp_corrected[pathHistory.icp_corrected.length - 1];
        drawRobot(ctx, pose, centerX, centerY, '#00ff00', '#00aa00');  // Green
    }
    if (showOdomPath && pathHistory.dead_reckoning.length > 0) {
        const pose = pathHistory.dead_reckoning[pathHistory.dead_reckoning.length - 1];
        drawRobot(ctx, pose, centerX, centerY, '#ff9500', '#cc7700');  // Orange
    }
    if (showArduinoPath && pathHistory.arduino_odo.length > 0) {
        const pose = pathHistory.arduino_odo[pathHistory.arduino_odo.length - 1];
        drawRobot(ctx, pose, centerX, centerY, '#00d4ff', '#0099cc');  // Blue
    }

    // Draw origin marker
    drawOrigin(ctx, centerX, centerY);

    // Draw legend
    drawLegend(ctx);

    // Continue rendering loop
    requestAnimationFrame(renderMapCanvas);
}

/**
 * Calculate auto-scale to fit traveled area
 */
function calculateAutoScale() {
    // Filter valid poses: exclude poses with unrealistic coordinates (> 100m suggests overflow/error)
    const MAX_REASONABLE_DISTANCE = 100.0;  // meters

    const validPoses = [
        ...pathHistory.dead_reckoning,
        ...pathHistory.arduino_odo,
        ...pathHistory.icp_corrected
    ].filter(pose => {
        const distance = Math.sqrt(pose.x * pose.x + pose.y * pose.y);
        return distance < MAX_REASONABLE_DISTANCE;
    });

    if (validPoses.length === 0) {
        viewScale = 100;  // Default: 100 pixels per meter
        return;
    }

    // Safety check: ensure canvas exists and has valid dimensions
    if (!mapCanvas || mapCanvas.width <= 0 || mapCanvas.height <= 0) {
        viewScale = 100;  // Default fallback
        return;
    }

    // Find bounding box
    let minX = 0, maxX = 0, minY = 0, maxY = 0;
    for (const pose of validPoses) {
        minX = Math.min(minX, pose.x);
        maxX = Math.max(maxX, pose.x);
        minY = Math.min(minY, pose.y);
        maxY = Math.max(maxY, pose.y);
    }

    // Ensure minimum view of 5m x 5m
    const rangeX = Math.max(maxX - minX, 5.0);
    const rangeY = Math.max(maxY - minY, 5.0);
    const maxRange = Math.max(rangeX, rangeY);

    // Calculate scale to fit in canvas with some padding
    const canvasSize = Math.min(mapCanvas.width, mapCanvas.height);
    const padding = 40;  // pixels
    const calculatedScale = (canvasSize - 2 * padding) / maxRange;

    // Ensure viewScale is always positive and finite
    viewScale = (calculatedScale > 0 && isFinite(calculatedScale)) ? calculatedScale : 100;
}

/**
 * Draw grid lines
 */
function drawGrid(ctx, width, height, centerX, centerY) {
    ctx.strokeStyle = '#333';
    ctx.lineWidth = 1;
    ctx.setLineDash([]);

    const gridSpacing = 1.0;  // 1 meter grid spacing
    const gridPixels = gridSpacing * viewScale;

    // Safety check: prevent infinite loops if gridPixels is invalid or too small
    // Minimum 5 pixels per grid line to prevent thousands of iterations
    if (gridPixels < 5 || !isFinite(gridPixels)) {
        return;
    }

    // Vertical lines
    for (let x = centerX; x < width; x += gridPixels) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, height);
        ctx.stroke();
    }
    for (let x = centerX - gridPixels; x > 0; x -= gridPixels) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, height);
        ctx.stroke();
    }

    // Horizontal lines
    for (let y = centerY; y < height; y += gridPixels) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();
    }
    for (let y = centerY - gridPixels; y > 0; y -= gridPixels) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();
    }

    // Draw grid labels
    ctx.fillStyle = '#666';
    ctx.font = '10px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';

    let labelIndex = 0;
    for (let x = centerX; x < width; x += gridPixels) {
        if (labelIndex !== 0) {
            ctx.fillText(`${labelIndex}m`, x, centerY + 5);
        }
        labelIndex++;
    }

    labelIndex = -1;
    for (let x = centerX - gridPixels; x > 0; x -= gridPixels) {
        ctx.fillText(`${labelIndex}m`, x, centerY + 5);
        labelIndex--;
    }
}

/**
 * Draw path
 */
function drawPath(ctx, path, color, centerX, centerY) {
    if (path.length < 2) return;

    ctx.strokeStyle = color;
    ctx.lineWidth = 3;  // Increased from 2 for better visibility
    ctx.setLineDash([]);

    // Optimization: downsample path if too many points
    // Draw at most every Nth point if path is very long
    const maxPoints = 500;
    const step = path.length > maxPoints ? Math.ceil(path.length / maxPoints) : 1;

    ctx.beginPath();
    let isFirst = true;
    for (let i = 0; i < path.length; i += step) {
        const pose = path[i];
        // Swap X and Y: robot's X becomes screen's Y, robot's Y becomes screen's X
        const x = centerX + pose.y * viewScale;  // Robot Y maps to screen X
        const y = centerY - pose.x * viewScale;  // Robot X maps to screen Y (negated, canvas Y down)

        if (isFirst) {
            ctx.moveTo(x, y);
            isFirst = false;
        } else {
            ctx.lineTo(x, y);
        }
    }
    // Always draw the last point to ensure path is complete
    if (step > 1 && path.length > 0) {
        const pose = path[path.length - 1];
        const x = centerX + pose.y * viewScale;
        const y = centerY - pose.x * viewScale;
        ctx.lineTo(x, y);
    }
    ctx.stroke();
}

/**
 * Draw robot as triangle
 */
function drawRobot(ctx, pose, centerX, centerY, fillColor, strokeColor) {
    // Swap X and Y: robot's X becomes screen's Y, robot's Y becomes screen's X
    const x = centerX + pose.y * viewScale;  // Robot Y maps to screen X
    const y = centerY - pose.x * viewScale;  // Robot X maps to screen Y (negated, canvas Y down)
    const size = 10;  // Triangle size in pixels

    ctx.save();
    ctx.translate(x, y);
    // Rotation: pose.theta (backend now produces correct sign: positive = right/clockwise)
    ctx.rotate(pose.theta);

    // Draw triangle (pointing up in robot frame = forward)
    ctx.fillStyle = fillColor;
    ctx.strokeStyle = strokeColor;
    ctx.lineWidth = 2;

    ctx.beginPath();
    ctx.moveTo(0, -size);           // Front point
    ctx.lineTo(-size * 0.6, size);  // Back left
    ctx.lineTo(size * 0.6, size);   // Back right
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    ctx.restore();
}

/**
 * Draw origin marker (start position)
 */
function drawOrigin(ctx, centerX, centerY) {
    // Draw crosshair at origin
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1;
    ctx.setLineDash([5, 5]);

    const crossSize = 15;

    // Horizontal line
    ctx.beginPath();
    ctx.moveTo(centerX - crossSize, centerY);
    ctx.lineTo(centerX + crossSize, centerY);
    ctx.stroke();

    // Vertical line
    ctx.beginPath();
    ctx.moveTo(centerX, centerY - crossSize);
    ctx.lineTo(centerX, centerY + crossSize);
    ctx.stroke();

    ctx.setLineDash([]);

    // Draw "START" label
    ctx.fillStyle = '#ffffff';
    ctx.font = '11px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    ctx.fillText('START', centerX, centerY + crossSize + 5);
}

/**
 * Draw legend
 */
function drawLegend(ctx) {
    const x = 10;
    const y = 10;
    const lineLength = 30;
    const spacing = 20;

    ctx.font = '12px monospace';
    ctx.textBaseline = 'middle';

    // Host odometry (dead reckoning)
    ctx.strokeStyle = '#ff9500';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x + lineLength, y);
    ctx.stroke();

    ctx.fillStyle = '#ff9500';
    ctx.textAlign = 'left';
    ctx.fillText('Host Odometry', x + lineLength + 5, y);

    // Arduino odometry
    ctx.strokeStyle = '#00d4ff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x, y + spacing);
    ctx.lineTo(x + lineLength, y + spacing);
    ctx.stroke();

    ctx.fillStyle = '#00d4ff';
    ctx.textAlign = 'left';
    ctx.fillText('Arduino Odo', x + lineLength + 5, y + spacing);

    // ICP-corrected
    ctx.strokeStyle = '#00ff00';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x, y + spacing * 2);
    ctx.lineTo(x + lineLength, y + spacing * 2);
    ctx.stroke();

    ctx.fillStyle = '#00ff00';
    ctx.textAlign = 'left';
    ctx.fillText('ICP-Corrected', x + lineLength + 5, y + spacing * 2);
}

// Update status periodically
setInterval(updateMapStatus, 500);

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    initializeMapVisualization();
});
