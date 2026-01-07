"""
LIDAR Data Processing Module

Processes and filters LIDAR scan data for use in SLAM algorithms.
Includes range filtering, outlier removal, coordinate transformations, and downsampling.
"""

import numpy as np
from typing import List, Tuple
from copy import deepcopy

try:
    from .sensor_data import LidarScan, LidarReading, Pose
except ImportError:
    from sensor_data import LidarScan, LidarReading, Pose


class LidarProcessor:
    """Process and filter LIDAR scan data

    Applies multiple filters to raw LIDAR data:
    1. Invalid readings removal (flagged by LIDAR sensor)
    2. Range filtering (min/max distance)
    3. Signal strength filtering (weak returns)
    4. Statistical outlier removal (local neighborhood analysis)

    Attributes:
        max_range_m: Maximum valid range in meters
        min_range_m: Minimum valid range in meters
        min_signal_strength: Minimum signal strength threshold
        outlier_threshold: Standard deviations for outlier detection
    """

    def __init__(
        self,
        max_range_m: float = 5.0,
        min_range_m: float = 0.1,
        min_signal_strength: int = 10,
        outlier_threshold: float = 3.0
    ):
        """Initialize LIDAR processor

        Args:
            max_range_m: Maximum valid range (meters)
            min_range_m: Minimum valid range (meters)
            min_signal_strength: Minimum signal strength (sensor units)
            outlier_threshold: Z-score threshold for outlier removal
        """
        self.max_range_m = max_range_m
        self.min_range_m = min_range_m
        self.min_signal_strength = min_signal_strength
        self.outlier_threshold = outlier_threshold

    def filter_scan(self, scan: LidarScan) -> LidarScan:
        """Apply all filters to LIDAR scan

        Filters applied in order:
        1. Remove invalid readings (flagged by sensor)
        2. Remove out-of-range readings
        3. Remove low signal strength readings
        4. Statistical outlier removal

        Args:
            scan: Raw LIDAR scan

        Returns:
            Filtered scan (invalid readings marked as such)
        """
        filtered_readings = []

        for reading in scan.readings:
            # Create copy to avoid modifying original
            r = LidarReading(
                angle_deg=reading.angle_deg,
                distance_mm=reading.distance_mm,
                signal_strength=reading.signal_strength,
                valid=reading.valid
            )

            # Filter 1: Already marked invalid by sensor
            if not r.valid:
                filtered_readings.append(r)
                continue

            # Filter 2: Range check
            dist_m = r.distance_mm / 1000.0
            if dist_m < self.min_range_m or dist_m > self.max_range_m:
                r.valid = False
                filtered_readings.append(r)
                continue

            # Filter 3: Low signal strength (sensor-specific tuning)
            if r.signal_strength < self.min_signal_strength:
                r.valid = False
                filtered_readings.append(r)
                continue

            filtered_readings.append(r)

        # Filter 4: Statistical outlier removal
        filtered_readings = self._remove_statistical_outliers(filtered_readings)

        return LidarScan(
            timestamp=scan.timestamp,
            rpm=scan.rpm,
            readings=filtered_readings
        )

    def _remove_statistical_outliers(
        self, readings: List[LidarReading]
    ) -> List[LidarReading]:
        """Remove outliers using local neighborhood statistics

        For each reading, compares distance to neighbors within a window.
        If reading differs by more than threshold standard deviations,
        marks it as invalid.

        Args:
            readings: List of LIDAR readings

        Returns:
            List with outliers marked invalid
        """
        if len(readings) < 5:
            return readings

        window_size = 5

        for i in range(len(readings)):
            if not readings[i].valid:
                continue

            # Get neighborhood window
            start = max(0, i - window_size // 2)
            end = min(len(readings), i + window_size // 2 + 1)

            # Collect valid neighbor distances
            neighbors = [
                r.distance_mm for r in readings[start:end]
                if r.valid and readings.index(r) != i
            ]

            if len(neighbors) < 3:
                continue

            # Compute local statistics
            mean = np.mean(neighbors)
            std = np.std(neighbors)

            if std == 0:
                continue

            # Check if current reading is statistical outlier
            z_score = abs(readings[i].distance_mm - mean) / std
            if z_score > self.outlier_threshold:
                readings[i].valid = False

        return readings

    def polar_to_cartesian(
        self, scan: LidarScan
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Convert LIDAR scan from polar to Cartesian coordinates

        Converts valid readings from (angle, distance) to (x, y) in robot frame.
        Robot frame: X forward, Y left, origin at robot center.

        Args:
            scan: LIDAR scan in polar coordinates

        Returns:
            Tuple of (points_xy, angles):
            - points_xy: Nx2 array of (x, y) coordinates in meters
            - angles: N-length array of angles in radians
        """
        valid_readings = [r for r in scan.readings if r.valid]

        if len(valid_readings) == 0:
            return np.zeros((0, 2)), np.zeros(0)

        points = []
        angles = []

        for reading in valid_readings:
            # Convert to radians
            angle_rad = np.radians(reading.angle_deg)

            # Convert to meters
            distance_m = reading.distance_mm / 1000.0

            # Polar to Cartesian (robot frame: X forward, Y left)
            x = distance_m * np.cos(angle_rad)
            y = distance_m * np.sin(angle_rad)

            points.append([x, y])
            angles.append(angle_rad)

        return np.array(points), np.array(angles)

    def transform_to_map_frame(
        self,
        points_robot: np.ndarray,
        robot_pose: Pose
    ) -> np.ndarray:
        """Transform points from robot frame to map frame

        Applies rigid 2D transformation (rotation + translation) to move points
        from robot-centric coordinates to global map coordinates.

        Args:
            points_robot: Nx2 array in robot frame
            robot_pose: Robot pose in map frame (x, y, theta)

        Returns:
            Nx2 array in map frame
        """
        if len(points_robot) == 0:
            return points_robot

        # Build 2D rotation matrix
        cos_theta = np.cos(robot_pose.theta)
        sin_theta = np.sin(robot_pose.theta)

        R = np.array([
            [cos_theta, -sin_theta],
            [sin_theta,  cos_theta]
        ])

        # Transform: p_map = R * p_robot + t
        points_map = (R @ points_robot.T).T
        points_map[:, 0] += robot_pose.x
        points_map[:, 1] += robot_pose.y

        return points_map

    def downsample(
        self,
        points: np.ndarray,
        voxel_size: float = 0.05
    ) -> np.ndarray:
        """Downsample point cloud using voxel grid

        Reduces point count by discretizing space into voxels and keeping
        one point per voxel. Useful for speeding up scan matching.

        Args:
            points: Nx2 array of points
            voxel_size: Voxel size in meters (default 5cm)

        Returns:
            Downsampled Mx2 array (M <= N)
        """
        if len(points) == 0:
            return points

        # Discretize points to voxel grid
        voxel_coords = np.floor(points / voxel_size).astype(int)

        # Find unique voxels (returns indices of first occurrence)
        unique_voxels, indices = np.unique(
            voxel_coords, axis=0, return_index=True
        )

        return points[indices]

    def get_scan_statistics(self, scan: LidarScan) -> dict:
        """Compute statistics about a LIDAR scan

        Args:
            scan: LIDAR scan to analyze

        Returns:
            Dictionary with scan statistics
        """
        valid_readings = [r for r in scan.readings if r.valid]

        if len(valid_readings) == 0:
            return {
                'total_readings': len(scan.readings),
                'valid_readings': 0,
                'valid_ratio': 0.0,
                'mean_distance_m': 0.0,
                'min_distance_m': 0.0,
                'max_distance_m': 0.0,
                'std_distance_m': 0.0,
            }

        distances = np.array([r.distance_mm / 1000.0 for r in valid_readings])

        return {
            'total_readings': len(scan.readings),
            'valid_readings': len(valid_readings),
            'valid_ratio': len(valid_readings) / len(scan.readings),
            'mean_distance_m': float(np.mean(distances)),
            'min_distance_m': float(np.min(distances)),
            'max_distance_m': float(np.max(distances)),
            'std_distance_m': float(np.std(distances)),
            'rpm': scan.rpm,
        }

    def __repr__(self) -> str:
        return (f"LidarProcessor(range=[{self.min_range_m},{self.max_range_m}]m, "
                f"min_strength={self.min_signal_strength}, "
                f"outlier_threshold={self.outlier_threshold}σ)")
