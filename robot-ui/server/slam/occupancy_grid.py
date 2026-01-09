"""
Occupancy Grid Mapping Module

Implements log-odds occupancy grid with Bresenham ray tracing for probabilistic
mapping from LIDAR scans. Uses log-odds representation for numerical stability
and efficient Bayesian updates.

Based on probabilistic robotics (Thrun, Burgard, Fox) occupancy grid mapping.
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
import math

try:
    from .sensor_data import Pose, LidarScan, LidarReading
except ImportError:
    from sensor_data import Pose, LidarScan, LidarReading


class OccupancyGrid:
    """2D probabilistic occupancy grid map using log-odds representation

    The grid uses Bayesian updates with log-odds for numerical stability:
    - log_odds = 0: Unknown (50% probability)
    - log_odds < 0: Free space
    - log_odds > 0: Occupied

    Probability conversion: p = 1 / (1 + exp(-log_odds))

    Attributes:
        width: Grid width in cells
        height: Grid height in cells
        resolution: Cell size in meters (e.g., 0.05 = 5cm)
        log_odds: Log-odds occupancy grid (2D numpy array)
        l_occ: Log-odds update for occupied cells
        l_free: Log-odds update for free cells
        l_max: Maximum log-odds (prevents saturation)
        l_min: Minimum log-odds (prevents saturation)
    """

    def __init__(
        self,
        width: int = 100,
        height: int = 100,
        resolution: float = 0.05,
        origin_offset_x: float = 0.0,
        origin_offset_y: float = 0.0,
        l_occ: float = 0.7,
        l_free: float = -0.4,
        l_max: float = 3.5,
        l_min: float = -3.5
    ):
        """Initialize occupancy grid

        Args:
            width: Grid width in cells (default 100 = 5m @ 5cm resolution)
            height: Grid height in cells (default 100 = 5m @ 5cm resolution)
            resolution: Cell size in meters (default 0.05 = 5cm/cell)
            origin_offset_x: X offset of world origin in map (meters, default 0.0 = centered)
            origin_offset_y: Y offset of world origin in map (meters, default 0.0 = centered)
            l_occ: Log-odds update for occupied cells (default 0.7)
            l_free: Log-odds update for free cells (default -0.4)
            l_max: Maximum log-odds clamp (default 3.5 ≈ 97% probability)
            l_min: Minimum log-odds clamp (default -3.5 ≈ 3% probability)

        Note on origin_offset:
            - (0, 0): Robot starts at center of map (symmetric coverage)
            - (-2.5, -2.5): Robot starts near bottom-left corner (good for room mapping)
            - Positive offset moves origin toward top-right of grid
        """
        self.width = width
        self.height = height
        self.resolution = resolution  # meters per cell
        self.origin_offset_x = origin_offset_x  # meters
        self.origin_offset_y = origin_offset_y  # meters

        # Log-odds grid: 0 = unknown, <0 = free, >0 = occupied
        self.log_odds = np.zeros((height, width), dtype=np.float32)

        # Update parameters (log-odds increments)
        self.l_occ = l_occ
        self.l_free = l_free
        self.l_max = l_max
        self.l_min = l_min

        # Statistics
        self.update_count = 0
        self.scan_count = 0

    def world_to_grid(self, x_m: float, y_m: float) -> Tuple[int, int]:
        """Convert world coordinates (meters) to grid indices

        World origin (0,0) maps to grid position based on origin_offset.
        With offset (0,0), world origin is at grid center.
        With offset (-2.5,-2.5), world origin is near bottom-left corner.

        Args:
            x_m: X coordinate in meters (world frame)
            y_m: Y coordinate in meters (world frame)

        Returns:
            (gx, gy): Grid indices
        """
        center_x = self.width // 2
        center_y = self.height // 2

        # Apply origin offset: subtract offset to shift origin position in grid
        gx = int(center_x + (x_m - self.origin_offset_x) / self.resolution)
        gy = int(center_y - (y_m - self.origin_offset_y) / self.resolution)  # Y-axis inverted (grid Y down)

        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates (meters)

        Args:
            gx: Grid X index
            gy: Grid Y index

        Returns:
            (x_m, y_m): World coordinates in meters
        """
        center_x = self.width // 2
        center_y = self.height // 2

        # Apply origin offset: add offset to restore world coordinates
        x_m = (gx - center_x) * self.resolution + self.origin_offset_x
        y_m = (center_y - gy) * self.resolution + self.origin_offset_y  # Y-axis inverted

        return x_m, y_m

    def is_valid_cell(self, gx: int, gy: int) -> bool:
        """Check if grid coordinates are within bounds

        Args:
            gx: Grid X index
            gy: Grid Y index

        Returns:
            True if cell is within grid bounds
        """
        return 0 <= gx < self.width and 0 <= gy < self.height

    def bresenham_ray(self, x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """Bresenham's line algorithm - returns all cells along line from (x0,y0) to (x1,y1)

        Efficient integer-only line drawing algorithm. Returns all grid cells
        that the line passes through, including start and end points.

        Args:
            x0, y0: Start point grid coordinates
            x1, y1: End point grid coordinates

        Returns:
            List of (gx, gy) tuples representing cells along the line
        """
        cells = []

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0

        while True:
            cells.append((x, y))

            if x == x1 and y == y1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return cells

    def update_from_scan(
        self,
        robot_pose: Pose,
        lidar_scan: LidarScan,
        max_range_m: float = 5.0,
        min_range_m: float = 0.1
    ):
        """Update occupancy grid from LIDAR scan using ray tracing

        For each valid LIDAR reading:
        1. Transform hit point from robot frame to world frame
        2. Trace ray from robot to hit point using Bresenham
        3. Update cells along ray as free space
        4. Update endpoint as occupied
        5. Clamp log-odds to prevent saturation

        Args:
            robot_pose: Current robot pose (x, y, theta)
            lidar_scan: LIDAR scan with readings
            max_range_m: Maximum valid range in meters (ignore beyond this)
            min_range_m: Minimum valid range in meters (ignore closer than this)
        """
        # Convert robot position to grid coordinates
        robot_gx, robot_gy = self.world_to_grid(robot_pose.x, robot_pose.y)

        if not self.is_valid_cell(robot_gx, robot_gy):
            # Robot outside grid bounds - cannot update
            return

        valid_readings = 0

        # Process each LIDAR reading
        for reading in lidar_scan.readings:
            # Skip invalid readings
            if not reading.valid:
                continue

            # Convert distance to meters
            dist_m = reading.distance_mm / 1000.0

            # Skip readings outside valid range
            if dist_m < min_range_m or dist_m > max_range_m:
                continue

            # Calculate beam angle in world frame
            # Note: LIDAR angle is in degrees, robot theta is in radians
            beam_angle_rad = math.radians(reading.angle_deg) + robot_pose.theta

            # Calculate hit point in world frame
            hit_x = robot_pose.x + dist_m * math.cos(beam_angle_rad)
            hit_y = robot_pose.y + dist_m * math.sin(beam_angle_rad)

            # Convert hit point to grid coordinates
            hit_gx, hit_gy = self.world_to_grid(hit_x, hit_y)

            # Skip if hit point is outside grid
            if not self.is_valid_cell(hit_gx, hit_gy):
                continue

            # Ray trace from robot to hit point
            ray_cells = self.bresenham_ray(robot_gx, robot_gy, hit_gx, hit_gy)

            # Update cells along ray
            for i, (gx, gy) in enumerate(ray_cells):
                if not self.is_valid_cell(gx, gy):
                    continue

                if i < len(ray_cells) - 1:
                    # Free space along ray (all cells except endpoint)
                    self.log_odds[gy, gx] += self.l_free
                else:
                    # Occupied cell at endpoint
                    self.log_odds[gy, gx] += self.l_occ

                # Clamp to prevent saturation
                self.log_odds[gy, gx] = np.clip(
                    self.log_odds[gy, gx],
                    self.l_min,
                    self.l_max
                )

            valid_readings += 1

        # Update statistics
        self.scan_count += 1
        self.update_count += valid_readings

    def get_probability_grid(self) -> np.ndarray:
        """Convert log-odds to probability: p = 1 / (1 + exp(-log_odds))

        Returns:
            2D numpy array of probabilities (0.0 to 1.0)
            - 0.0: Definitely free
            - 0.5: Unknown
            - 1.0: Definitely occupied
        """
        return 1.0 / (1.0 + np.exp(-self.log_odds))

    def get_display_grid(self) -> np.ndarray:
        """Convert log-odds to discrete display values

        Returns:
            2D numpy array of uint8 values:
            - 0: Free (probability < 0.3)
            - 1: Unknown (probability 0.3-0.7)
            - 2: Occupied (probability > 0.7)
        """
        prob_grid = self.get_probability_grid()

        display_grid = np.zeros_like(prob_grid, dtype=np.uint8)
        display_grid[prob_grid < 0.3] = 0  # Free
        display_grid[(prob_grid >= 0.3) & (prob_grid <= 0.7)] = 1  # Unknown
        display_grid[prob_grid > 0.7] = 2  # Occupied

        return display_grid

    def serialize_for_ui(self, robot_pose: Optional[Pose] = None) -> Dict:
        """Return JSON-serializable format for WebSocket transmission

        Args:
            robot_pose: Optional current robot pose to include

        Returns:
            Dictionary with grid data:
            {
                'width': 100,
                'height': 100,
                'resolution': 0.05,
                'grid': [[0, 1, 2, ...], ...],  # 2D list
                'robot_pose': {'x': 1.2, 'y': 0.5, 'theta': 0.78} or None,
                'stats': {'scans': 42, 'updates': 1234}
            }
        """
        display_grid = self.get_display_grid()

        result = {
            'width': self.width,
            'height': self.height,
            'resolution': self.resolution,
            'origin_offset_x': self.origin_offset_x,
            'origin_offset_y': self.origin_offset_y,
            'grid': display_grid.tolist(),  # Convert numpy array to list for JSON
            'stats': {
                'scans': self.scan_count,
                'updates': self.update_count
            }
        }

        # Add robot pose if provided
        if robot_pose is not None:
            result['robot_pose'] = {
                'x': robot_pose.x,
                'y': robot_pose.y,
                'theta': robot_pose.theta
            }
        else:
            result['robot_pose'] = None

        return result

    def clear(self):
        """Reset grid to unknown (all cells = 0)"""
        self.log_odds.fill(0.0)
        self.scan_count = 0
        self.update_count = 0

    def get_stats(self) -> Dict:
        """Get mapping statistics

        Returns:
            Dictionary with stats:
            {
                'total_cells': 10000,
                'free_cells': 234,
                'occupied_cells': 56,
                'unknown_cells': 9710,
                'scans_processed': 42,
                'updates_total': 1234
            }
        """
        display_grid = self.get_display_grid()

        return {
            'total_cells': self.width * self.height,
            'free_cells': int(np.sum(display_grid == 0)),
            'occupied_cells': int(np.sum(display_grid == 2)),
            'unknown_cells': int(np.sum(display_grid == 1)),
            'scans_processed': self.scan_count,
            'updates_total': self.update_count
        }

    def __repr__(self) -> str:
        stats = self.get_stats()
        return (
            f"OccupancyGrid({self.width}×{self.height} @ {self.resolution*1000:.0f}mm/cell, "
            f"{stats['scans_processed']} scans, "
            f"{stats['occupied_cells']} occupied, "
            f"{stats['free_cells']} free)"
        )
