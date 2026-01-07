"""
Sensor Data Classes

Data structures for representing sensor readings from the robot.
All timestamps are in milliseconds since ESP32 boot.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict
import numpy as np


@dataclass
class OdomReading:
    """Single odometry reading from robot

    Attributes:
        timestamp: Milliseconds since ESP32 boot
        delta_left: Encoder ticks since last reading (left wheel)
        delta_right: Encoder ticks since last reading (right wheel)
        x_mm: X position in millimeters (robot frame origin)
        y_mm: Y position in millimeters (robot frame origin)
        theta_mrad: Heading in milliradians (robot frame origin)
        vel_left: Left wheel velocity in m/s
        vel_right: Right wheel velocity in m/s
    """
    timestamp: float
    delta_left: int
    delta_right: int
    x_mm: int
    y_mm: int
    theta_mrad: int
    vel_left: float
    vel_right: float

    @property
    def timestamp_sec(self) -> float:
        """Convert timestamp to seconds

        Returns:
            Timestamp in seconds
        """
        return self.timestamp / 1000.0

    @property
    def x_m(self) -> float:
        """X position in meters

        Returns:
            X coordinate in meters
        """
        return self.x_mm / 1000.0

    @property
    def y_m(self) -> float:
        """Y position in meters

        Returns:
            Y coordinate in meters
        """
        return self.y_mm / 1000.0

    @property
    def theta_rad(self) -> float:
        """Heading in radians

        Returns:
            Heading angle in radians
        """
        return self.theta_mrad / 1000.0

    @property
    def linear_velocity(self) -> float:
        """Calculate linear velocity (average of wheels)

        Returns:
            Linear velocity in m/s
        """
        return (self.vel_left + self.vel_right) / 2.0

    @property
    def angular_velocity(self) -> float:
        """Calculate angular velocity

        Assumes differential drive with 0.24m wheelbase.

        Returns:
            Angular velocity in rad/s
        """
        WHEELBASE = 0.24  # meters
        return (self.vel_right - self.vel_left) / WHEELBASE

    def to_dict(self) -> Dict:
        """Convert to dictionary for JSON serialization

        Returns:
            Dictionary representation
        """
        return {
            'timestamp': self.timestamp,
            'timestamp_sec': self.timestamp_sec,
            'delta_left': self.delta_left,
            'delta_right': self.delta_right,
            'x_mm': self.x_mm,
            'y_mm': self.y_mm,
            'theta_mrad': self.theta_mrad,
            'x_m': self.x_m,
            'y_m': self.y_m,
            'theta_rad': self.theta_rad,
            'vel_left': self.vel_left,
            'vel_right': self.vel_right,
            'linear_velocity': self.linear_velocity,
            'angular_velocity': self.angular_velocity,
        }

    def __repr__(self) -> str:
        return (f"OdomReading(t={self.timestamp:.0f}ms, "
                f"pos=({self.x_m:.3f}, {self.y_m:.3f})m, "
                f"θ={self.theta_rad:.3f}rad, "
                f"v={self.linear_velocity:.3f}m/s)")


@dataclass
class LidarReading:
    """Single LIDAR distance measurement

    Attributes:
        angle_deg: Angle in degrees (0-359)
        distance_mm: Distance in millimeters
        signal_strength: Signal strength (0-65535)
        valid: Reading is valid (not flagged as invalid)
    """
    angle_deg: float
    distance_mm: int
    signal_strength: int
    valid: bool

    @property
    def distance_m(self) -> float:
        """Distance in meters

        Returns:
            Distance in meters
        """
        return self.distance_mm / 1000.0

    @property
    def angle_rad(self) -> float:
        """Angle in radians

        Returns:
            Angle in radians
        """
        return np.deg2rad(self.angle_deg)

    def to_cartesian(self) -> tuple[float, float]:
        """Convert polar coordinates to Cartesian (robot frame)

        Returns:
            (x, y) tuple in meters, robot frame (x=forward, y=left)
        """
        if not self.valid or self.distance_mm == 0:
            return (0.0, 0.0)

        # LIDAR frame: 0° = forward (X), 90° = left (Y)
        x = self.distance_m * np.cos(self.angle_rad)
        y = self.distance_m * np.sin(self.angle_rad)
        return (x, y)

    def to_dict(self) -> Dict:
        """Convert to dictionary for JSON serialization

        Returns:
            Dictionary representation
        """
        return {
            'angle_deg': self.angle_deg,
            'angle_rad': self.angle_rad,
            'distance_mm': self.distance_mm,
            'distance_m': self.distance_m,
            'signal_strength': self.signal_strength,
            'valid': self.valid,
        }


@dataclass
class LidarScan:
    """Complete or partial LIDAR scan

    In the current system, LIDAR data arrives in batches of ~40 readings
    per message (10 Neato packets × 4 readings). A full 360° scan consists
    of multiple batches.

    Attributes:
        timestamp: Milliseconds when scan data was received
        rpm: LIDAR rotation speed in RPM
        readings: List of LIDAR readings (may be partial scan)
        robot_pose: Robot pose when scan was taken (optional, for Phase 3+)
    """
    timestamp: float
    rpm: int
    readings: List[LidarReading]
    robot_pose: Optional['Pose'] = None

    @property
    def timestamp_sec(self) -> float:
        """Convert timestamp to seconds

        Returns:
            Timestamp in seconds
        """
        return self.timestamp / 1000.0

    @property
    def num_readings(self) -> int:
        """Number of readings in this scan

        Returns:
            Count of readings
        """
        return len(self.readings)

    @property
    def num_valid_readings(self) -> int:
        """Count valid readings

        Returns:
            Count of valid (non-flagged) readings
        """
        return sum(1 for r in self.readings if r.valid)

    @property
    def angular_coverage_deg(self) -> float:
        """Angular coverage of this scan

        Returns:
            Angular span in degrees
        """
        if len(self.readings) < 2:
            return 0.0

        angles = [r.angle_deg for r in self.readings]
        return max(angles) - min(angles)

    def get_cartesian_points(self, valid_only: bool = True) -> np.ndarray:
        """Convert to Nx2 array of (x,y) points in robot frame

        Args:
            valid_only: Only include valid readings

        Returns:
            Nx2 numpy array of (x, y) coordinates in meters
        """
        points = []
        for reading in self.readings:
            if valid_only and not reading.valid:
                continue
            if reading.distance_mm == 0:  # Skip zero distance
                continue

            x, y = reading.to_cartesian()
            points.append([x, y])

        if len(points) == 0:
            return np.empty((0, 2))

        return np.array(points)

    def get_angle_range(self) -> tuple[float, float]:
        """Get min and max angles covered by this scan

        Returns:
            (min_angle, max_angle) in degrees
        """
        if len(self.readings) == 0:
            return (0.0, 0.0)

        angles = [r.angle_deg for r in self.readings]
        return (min(angles), max(angles))

    def to_dict(self) -> Dict:
        """Convert to dictionary for JSON serialization

        Returns:
            Dictionary representation
        """
        return {
            'timestamp': self.timestamp,
            'timestamp_sec': self.timestamp_sec,
            'rpm': self.rpm,
            'num_readings': self.num_readings,
            'num_valid_readings': self.num_valid_readings,
            'angular_coverage_deg': self.angular_coverage_deg,
            'readings': [r.to_dict() for r in self.readings],
            'robot_pose': self.robot_pose.to_dict() if self.robot_pose else None,
        }

    def __repr__(self) -> str:
        angle_min, angle_max = self.get_angle_range()
        return (f"LidarScan(t={self.timestamp:.0f}ms, "
                f"rpm={self.rpm}, "
                f"readings={self.num_readings} ({self.num_valid_readings} valid), "
                f"angles={angle_min:.0f}°-{angle_max:.0f}°)")


@dataclass
class Pose:
    """Robot pose in map frame

    Attributes:
        x: X coordinate in meters
        y: Y coordinate in meters
        theta: Heading angle in radians
        timestamp: Timestamp in seconds
    """
    x: float
    y: float
    theta: float
    timestamp: float

    @property
    def position(self) -> np.ndarray:
        """Position as 2D numpy array

        Returns:
            [x, y] array
        """
        return np.array([self.x, self.y])

    def to_transform_matrix(self) -> np.ndarray:
        """Convert to 3x3 homogeneous transformation matrix

        Returns:
            3x3 transformation matrix:
            [cos(θ)  -sin(θ)   x]
            [sin(θ)   cos(θ)   y]
            [  0        0      1]
        """
        cos_theta = np.cos(self.theta)
        sin_theta = np.sin(self.theta)

        return np.array([
            [cos_theta, -sin_theta, self.x],
            [sin_theta,  cos_theta, self.y],
            [0.0,        0.0,       1.0]
        ])

    def transform_point(self, point: np.ndarray) -> np.ndarray:
        """Transform a point from robot frame to map frame

        Args:
            point: [x, y] point in robot frame

        Returns:
            [x, y] point in map frame
        """
        # Homogeneous coordinates
        point_h = np.array([point[0], point[1], 1.0])

        # Transform
        T = self.to_transform_matrix()
        result_h = T @ point_h

        return result_h[:2]

    def distance_to(self, other: 'Pose') -> float:
        """Euclidean distance to another pose

        Args:
            other: Another pose

        Returns:
            Distance in meters
        """
        dx = self.x - other.x
        dy = self.y - other.y
        return np.sqrt(dx*dx + dy*dy)

    def angle_difference(self, other: 'Pose') -> float:
        """Angular difference to another pose (wrapped to [-π, π])

        Args:
            other: Another pose

        Returns:
            Angle difference in radians
        """
        diff = other.theta - self.theta

        # Wrap to [-π, π]
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi

        return diff

    def to_dict(self) -> Dict:
        """Convert to dictionary for JSON serialization

        Returns:
            Dictionary representation
        """
        return {
            'x': self.x,
            'y': self.y,
            'theta': self.theta,
            'theta_deg': np.rad2deg(self.theta),
            'timestamp': self.timestamp,
        }

    def __repr__(self) -> str:
        return (f"Pose(x={self.x:.3f}m, y={self.y:.3f}m, "
                f"θ={self.theta:.3f}rad, t={self.timestamp:.3f}s)")


@dataclass
class SyncedData:
    """LIDAR scan synchronized with interpolated odometry

    This class represents a LIDAR scan with the robot's odometry state
    interpolated to the exact timestamp of the scan. This is crucial for
    accurate SLAM, as it ensures we know exactly where the robot was when
    each scan was taken.

    Attributes:
        lidar_scan: The LIDAR scan data
        odom_at_scan: Interpolated odometry at scan timestamp
        odom_before: Last odometry reading before scan
        odom_after: First odometry reading after scan
        interpolation_weight: Weight used for interpolation [0, 1]
    """
    lidar_scan: LidarScan
    odom_at_scan: OdomReading
    odom_before: OdomReading
    odom_after: OdomReading
    interpolation_weight: float

    @property
    def timestamp(self) -> float:
        """Get scan timestamp (in milliseconds)

        Returns:
            Timestamp in milliseconds
        """
        return self.lidar_scan.timestamp

    @property
    def timestamp_sec(self) -> float:
        """Get scan timestamp (in seconds)

        Returns:
            Timestamp in seconds
        """
        return self.lidar_scan.timestamp_sec

    @property
    def interpolation_quality(self) -> str:
        """Assess interpolation quality based on time gap

        Returns:
            Quality string: 'excellent', 'good', 'fair', 'poor'
        """
        time_gap_ms = self.odom_after.timestamp - self.odom_before.timestamp

        if time_gap_ms <= 100:  # Within one 10Hz sample
            return 'excellent'
        elif time_gap_ms <= 200:  # Within two samples
            return 'good'
        elif time_gap_ms <= 500:  # Within half second
            return 'fair'
        else:
            return 'poor'

    def to_dict(self) -> Dict:
        """Convert to dictionary for JSON serialization

        Returns:
            Dictionary representation
        """
        return {
            'timestamp': self.timestamp,
            'timestamp_sec': self.timestamp_sec,
            'interpolation_weight': self.interpolation_weight,
            'interpolation_quality': self.interpolation_quality,
            'lidar_scan': self.lidar_scan.to_dict(),
            'odom_at_scan': self.odom_at_scan.to_dict(),
            'odom_before': self.odom_before.to_dict(),
            'odom_after': self.odom_after.to_dict(),
        }

    def __repr__(self) -> str:
        return (f"SyncedData(t={self.timestamp:.0f}ms, "
                f"interp_weight={self.interpolation_weight:.3f}, "
                f"quality={self.interpolation_quality}, "
                f"lidar_readings={self.lidar_scan.num_readings})")
