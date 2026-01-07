/**
 * Map Canvas - Robot Path Visualization
 * Shows robot position and traveled path relative to start position
 * Displays both dead reckoning (orange) and ICP-corrected (green) paths
 */

// Map state
let mapCanvas = null;
let mapCtx = null;
let pathHistory = {
    dead_reckoning: [],  // Array of {x, y, theta} poses
    icp_corrected: []    // Array of {x, y, theta} poses
};
const MAX_HISTORY = 1000;
let lastMapUpdate = 0;

// View parameters
let viewScale = 100;  // pixels per meter (will auto-scale)
let viewOffsetX = 0;  // pan offset in pixels
let viewOffsetY = 0;

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
    const icpCount = pathHistory.icp_corrected.length;

    if (drCount === 0 && icpCount === 0) {
        statusElement.textContent = 'No data';
        statusElement.className = 'status-text status-inactive';
    } else {
        const age = Date.now() - lastMapUpdate;
        const ageText = age < 1000 ? 'Live' : `${(age / 1000).toFixed(1)}s ago`;
        statusElement.textContent = `${Math.max(drCount, icpCount)} poses | ${ageText}`;
        statusElement.className = 'status-text status-active';
    }
}

/**
 * Render map canvas (runs continuously via requestAnimationFrame)
 */
function renderMapCanvas() {
    if (!mapCtx || !mapCanvas) return;

    const ctx = mapCtx;
    const width = mapCanvas.width;
    const height = mapCanvas.height;
    const centerX = width / 2;
    const centerY = height / 2;

    // Clear canvas
    ctx.fillStyle = '#1a1a1a';
    ctx.fillRect(0, 0, width, height);

    // Calculate auto-scale based on traveled area
    calculateAutoScale();

    // Draw grid
    drawGrid(ctx, width, height, centerX, centerY);

    // Draw paths
    drawPath(ctx, pathHistory.dead_reckoning, '#ff9500', centerX, centerY);  // Orange
    drawPath(ctx, pathHistory.icp_corrected, '#00ff00', centerX, centerY);   // Green

    // Draw current robot pose (use ICP-corrected if available, else dead reckoning)
    const currentPose = pathHistory.icp_corrected.length > 0
        ? pathHistory.icp_corrected[pathHistory.icp_corrected.length - 1]
        : pathHistory.dead_reckoning.length > 0
            ? pathHistory.dead_reckoning[pathHistory.dead_reckoning.length - 1]
            : null;

    if (currentPose) {
        drawRobot(ctx, currentPose, centerX, centerY);
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
    // Combine all poses
    const allPoses = [...pathHistory.dead_reckoning, ...pathHistory.icp_corrected];

    if (allPoses.length === 0) {
        viewScale = 100;  // Default: 100 pixels per meter
        return;
    }

    // Find bounding box
    let minX = 0, maxX = 0, minY = 0, maxY = 0;
    for (const pose of allPoses) {
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
    viewScale = (canvasSize - 2 * padding) / maxRange;
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
    ctx.lineWidth = 2;
    ctx.setLineDash([]);

    ctx.beginPath();
    for (let i = 0; i < path.length; i++) {
        const pose = path[i];
        const x = centerX + pose.x * viewScale;
        const y = centerY - pose.y * viewScale;  // Flip Y (canvas Y down, map Y up)

        if (i === 0) {
            ctx.moveTo(x, y);
        } else {
            ctx.lineTo(x, y);
        }
    }
    ctx.stroke();
}

/**
 * Draw robot as triangle
 */
function drawRobot(ctx, pose, centerX, centerY) {
    const x = centerX + pose.x * viewScale;
    const y = centerY - pose.y * viewScale;  // Flip Y
    const size = 10;  // Triangle size in pixels

    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(-pose.theta);  // Negative because canvas Y is flipped

    // Draw triangle (pointing up in robot frame = forward)
    ctx.fillStyle = '#00ffff';  // Cyan
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1;

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

    // Dead reckoning
    ctx.strokeStyle = '#ff9500';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x + lineLength, y);
    ctx.stroke();

    ctx.fillStyle = '#ff9500';
    ctx.textAlign = 'left';
    ctx.fillText('Dead Reckoning', x + lineLength + 5, y);

    // ICP-corrected
    ctx.strokeStyle = '#00ff00';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x, y + spacing);
    ctx.lineTo(x + lineLength, y + spacing);
    ctx.stroke();

    ctx.fillStyle = '#00ff00';
    ctx.textAlign = 'left';
    ctx.fillText('ICP-Corrected', x + lineLength + 5, y + spacing);
}

// Update status periodically
setInterval(updateMapStatus, 500);

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    initializeMapVisualization();
});
