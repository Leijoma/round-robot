/**
 * Occupancy Grid Map Canvas Renderer
 *
 * Renders a 2D occupancy grid map on HTML5 canvas showing:
 * - Free space (white)
 * - Unknown space (gray)
 * - Occupied space (black)
 * - Robot position and orientation (green triangle)
 * - Grid lines for reference (1m spacing)
 *
 * Uses data from occupancy grid backend via WebSocket 'map_update' events.
 */

class OccupancyMapRenderer {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        if (!this.canvas) {
            console.error(`Canvas element '${canvasId}' not found`);
            return;
        }

        this.ctx = this.canvas.getContext('2d');

        // Grid dimensions (will be updated from map data)
        this.width = 100;
        this.height = 100;
        this.resolution = 0.05;  // meters per cell

        // Current map data
        this.grid = null;
        this.robotPose = null;

        // Colors
        this.colors = {
            free: '#FFFFFF',        // White - free space
            unknown: '#C0C0C0',     // Light gray - unknown
            occupied: '#000000',    // Black - obstacles
            robot: '#00FF00',       // Green - robot
            robotOutline: '#008000', // Dark green - robot outline
            gridLines: '#E0E0E0',   // Very light gray - grid lines
            centerCross: '#FF0000', // Red - origin marker
            background: '#F5F5F5'   // Off-white background
        };

        // Rendering settings
        this.showGridLines = true;
        this.showOrigin = true;
        this.robotSize = 15;  // pixels

        // Statistics
        this.lastUpdateTime = 0;
        this.updateCount = 0;

        // Initialize with empty grid
        this.clear();
    }

    /**
     * Update the map with new grid data
     * @param {Object} mapData - Map data from server
     *   {width, height, resolution, grid[][], robot_pose{x,y,theta}, stats}
     */
    updateMap(mapData) {
        if (!mapData || !mapData.grid) {
            console.warn('Invalid map data received');
            return;
        }

        // Update grid parameters
        this.width = mapData.width;
        this.height = mapData.height;
        this.resolution = mapData.resolution;
        this.originOffsetX = mapData.origin_offset_x || 0.0;
        this.originOffsetY = mapData.origin_offset_y || 0.0;
        this.grid = mapData.grid;
        this.robotPose = mapData.robot_pose;

        // Update statistics
        this.lastUpdateTime = Date.now();
        this.updateCount++;

        // Render the map
        this.render();
    }

    /**
     * Render the complete map
     */
    render() {
        if (!this.grid) {
            return;
        }

        // Clear canvas with background color
        this.ctx.fillStyle = this.colors.background;
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);

        // Calculate cell size in pixels
        const cellWidth = this.canvas.width / this.width;
        const cellHeight = this.canvas.height / this.height;

        // Draw occupancy grid
        this.drawGrid(cellWidth, cellHeight);

        // Draw grid lines (1m spacing)
        if (this.showGridLines) {
            this.drawGridLines(cellWidth, cellHeight);
        }

        // Draw origin marker
        if (this.showOrigin) {
            this.drawOrigin(cellWidth, cellHeight);
        }

        // Draw robot pose
        if (this.robotPose) {
            this.drawRobot(this.robotPose, cellWidth, cellHeight);
        }
    }

    /**
     * Draw the occupancy grid cells
     */
    drawGrid(cellWidth, cellHeight) {
        for (let gy = 0; gy < this.height; gy++) {
            for (let gx = 0; gx < this.width; gx++) {
                const cellValue = this.grid[gy][gx];

                // Set color based on cell value
                // 0 = free, 1 = unknown, 2 = occupied
                if (cellValue === 0) {
                    this.ctx.fillStyle = this.colors.free;
                } else if (cellValue === 1) {
                    this.ctx.fillStyle = this.colors.unknown;
                } else if (cellValue === 2) {
                    this.ctx.fillStyle = this.colors.occupied;
                } else {
                    // Unknown value - default to unknown
                    this.ctx.fillStyle = this.colors.unknown;
                }

                // Draw cell
                this.ctx.fillRect(
                    gx * cellWidth,
                    gy * cellHeight,
                    cellWidth,
                    cellHeight
                );
            }
        }
    }

    /**
     * Draw grid lines at 1m spacing
     */
    drawGridLines(cellWidth, cellHeight) {
        const gridSpacing = 1.0 / this.resolution;  // 1m in cells (20 cells @ 5cm)

        this.ctx.strokeStyle = this.colors.gridLines;
        this.ctx.lineWidth = 1;
        this.ctx.setLineDash([]);

        // Vertical lines
        for (let i = 0; i <= this.width; i += gridSpacing) {
            const x = i * cellWidth;
            this.ctx.beginPath();
            this.ctx.moveTo(x, 0);
            this.ctx.lineTo(x, this.canvas.height);
            this.ctx.stroke();
        }

        // Horizontal lines
        for (let i = 0; i <= this.height; i += gridSpacing) {
            const y = i * cellHeight;
            this.ctx.beginPath();
            this.ctx.moveTo(0, y);
            this.ctx.lineTo(this.canvas.width, y);
            this.ctx.stroke();
        }
    }

    /**
     * Draw origin marker (center of map)
     */
    drawOrigin(cellWidth, cellHeight) {
        const centerX = this.canvas.width / 2;
        const centerY = this.canvas.height / 2;
        const crossSize = 10;

        this.ctx.strokeStyle = this.colors.centerCross;
        this.ctx.lineWidth = 2;
        this.ctx.setLineDash([]);

        // Draw cross
        this.ctx.beginPath();
        this.ctx.moveTo(centerX - crossSize, centerY);
        this.ctx.lineTo(centerX + crossSize, centerY);
        this.ctx.moveTo(centerX, centerY - crossSize);
        this.ctx.lineTo(centerX, centerY + crossSize);
        this.ctx.stroke();
    }

    /**
     * Draw robot pose as triangle
     * @param {Object} pose - Robot pose {x, y, theta}
     * @param {number} cellWidth - Cell width in pixels
     * @param {number} cellHeight - Cell height in pixels
     */
    drawRobot(pose, cellWidth, cellHeight) {
        // Convert world coordinates to pixel coordinates
        const centerX = this.canvas.width / 2;
        const centerY = this.canvas.height / 2;

        // Grid coordinates (account for origin offset)
        const gx = this.width / 2 + (pose.x - this.originOffsetX) / this.resolution;
        const gy = this.height / 2 - (pose.y - this.originOffsetY) / this.resolution;  // Y inverted

        // Pixel coordinates
        const pixelX = gx * cellWidth;
        const pixelY = gy * cellHeight;

        // Draw robot as triangle pointing in direction of heading
        this.ctx.save();
        this.ctx.translate(pixelX, pixelY);
        this.ctx.rotate(-pose.theta);  // Negative because canvas Y is down

        // Draw filled triangle
        this.ctx.fillStyle = this.colors.robot;
        this.ctx.beginPath();
        this.ctx.moveTo(this.robotSize, 0);           // Front point
        this.ctx.lineTo(-this.robotSize/2, -this.robotSize/2);  // Back left
        this.ctx.lineTo(-this.robotSize/2, this.robotSize/2);   // Back right
        this.ctx.closePath();
        this.ctx.fill();

        // Draw outline
        this.ctx.strokeStyle = this.colors.robotOutline;
        this.ctx.lineWidth = 2;
        this.ctx.stroke();

        this.ctx.restore();
    }

    /**
     * Clear the canvas
     */
    clear() {
        this.ctx.fillStyle = this.colors.background;
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);

        // Draw "No Data" message
        this.ctx.fillStyle = '#666666';
        this.ctx.font = '16px Arial';
        this.ctx.textAlign = 'center';
        this.ctx.textBaseline = 'middle';
        this.ctx.fillText(
            'No Map Data',
            this.canvas.width / 2,
            this.canvas.height / 2
        );
    }

    /**
     * Toggle grid lines visibility
     */
    toggleGridLines() {
        this.showGridLines = !this.showGridLines;
        this.render();
    }

    /**
     * Toggle origin marker visibility
     */
    toggleOrigin() {
        this.showOrigin = !this.showOrigin;
        this.render();
    }

    /**
     * Set robot marker size
     * @param {number} size - Size in pixels
     */
    setRobotSize(size) {
        this.robotSize = size;
        this.render();
    }

    /**
     * Get rendering statistics
     * @returns {Object} Statistics
     */
    getStats() {
        return {
            updateCount: this.updateCount,
            lastUpdateTime: this.lastUpdateTime,
            gridSize: `${this.width}×${this.height}`,
            resolution: this.resolution,
            coverage: `${(this.width * this.resolution).toFixed(1)}m × ${(this.height * this.resolution).toFixed(1)}m`
        };
    }

    /**
     * Convert canvas pixel coordinates to world coordinates
     * Useful for mouse interaction
     * @param {number} pixelX - Canvas X coordinate
     * @param {number} pixelY - Canvas Y coordinate
     * @returns {Object} World coordinates {x, y}
     */
    pixelToWorld(pixelX, pixelY) {
        const cellWidth = this.canvas.width / this.width;
        const cellHeight = this.canvas.height / this.height;

        const gx = pixelX / cellWidth;
        const gy = pixelY / cellHeight;

        // Account for origin offset
        const worldX = (gx - this.width / 2) * this.resolution + this.originOffsetX;
        const worldY = (this.height / 2 - gy) * this.resolution + this.originOffsetY;

        return { x: worldX, y: worldY };
    }

    /**
     * Get cell value at pixel coordinates
     * @param {number} pixelX - Canvas X coordinate
     * @param {number} pixelY - Canvas Y coordinate
     * @returns {number|null} Cell value (0=free, 1=unknown, 2=occupied) or null if out of bounds
     */
    getCellAtPixel(pixelX, pixelY) {
        if (!this.grid) {
            return null;
        }

        const cellWidth = this.canvas.width / this.width;
        const cellHeight = this.canvas.height / this.height;

        const gx = Math.floor(pixelX / cellWidth);
        const gy = Math.floor(pixelY / cellHeight);

        if (gx < 0 || gx >= this.width || gy < 0 || gy >= this.height) {
            return null;
        }

        return this.grid[gy][gx];
    }
}

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
    module.exports = OccupancyMapRenderer;
}
