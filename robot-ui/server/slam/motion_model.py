"""
Motion Model for Differential Drive Robot

Implements forward and inverse kinematics for a differential drive robot.
Handles straight line motion, curved motion, and motion uncertainty.

Physical Parameters (from robot specifications):
- Wheel diameter: 80mm
- Wheelbase (distance between wheels): 244mm
- Encoder ticks per revolution: 360
- LIDAR offset: 10mm forward from wheel axle centerline
"""

from dataclasses import dataclass, asdict
import numpy as np
from typing import Tuple, Dict
import json
import os

try:
    from .sensor_data import Pose
except ImportError:
    from sensor_data import Pose


@dataclass
class RobotParameters:
    """Physical parameters of the differential drive robot

    Attributes:
        wheel_diameter: Diameter of drive wheels in meters
        wheelbase: Distance between left and right wheels in meters
        ticks_per_revolution: Encoder ticks per wheel revolution
        lidar_offset_x: LIDAR offset forward from wheel axle (meters, positive = forward)
        lidar_offset_y: LIDAR offset lateral from centerline (meters, positive = right)
    """
    wheel_diameter: float = 0.080  # meters (80mm)
    wheelbase: float = 0.244  # meters (244mm)
    ticks_per_revolution: int = 360
    lidar_offset_x: float = 0.010  # meters (10mm forward)
    lidar_offset_y: float = 0.0  # meters (centered laterally)

    @property
    def wheel_radius(self) -> float:
        """Wheel radius in meters

        Returns:
            Radius in meters
        """
        return self.wheel_diameter / 2.0

    @property
    def distance_per_tick(self) -> float:
        """Linear distance traveled per encoder tick

        Returns:
            Distance in meters per tick
        """
        circumference = np.pi * self.wheel_diameter
        return circumference / self.ticks_per_revolution

    def to_dict(self) -> Dict:
        """Convert to dictionary for JSON serialization

        Returns:
            Dictionary representation
        """
        return {
            'wheel_diameter': self.wheel_diameter,
            'wheelbase': self.wheelbase,
            'ticks_per_revolution': self.ticks_per_revolution,
            'lidar_offset_x': self.lidar_offset_x,
            'lidar_offset_y': self.lidar_offset_y,
            'wheel_diameter_mm': self.wheel_diameter * 1000,
            'wheelbase_mm': self.wheelbase * 1000,
            'lidar_offset_x_mm': self.lidar_offset_x * 1000,
            'lidar_offset_y_mm': self.lidar_offset_y * 1000,
        }

    @classmethod
    def from_dict(cls, data: Dict) -> 'RobotParameters':
        """Create from dictionary

        Args:
            data: Dictionary with parameters

        Returns:
            RobotParameters instance
        """
        return cls(
            wheel_diameter=data.get('wheel_diameter', 0.080),
            wheelbase=data.get('wheelbase', 0.244),
            ticks_per_revolution=data.get('ticks_per_revolution', 360),
            lidar_offset_x=data.get('lidar_offset_x', 0.010),
            lidar_offset_y=data.get('lidar_offset_y', 0.0),
        )

    def save(self, filepath: str) -> None:
        """Save parameters to JSON file

        Args:
            filepath: Path to save file
        """
        with open(filepath, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load(cls, filepath: str) -> 'RobotParameters':
        """Load parameters from JSON file

        Args:
            filepath: Path to load from

        Returns:
            RobotParameters instance
        """
        if not os.path.exists(filepath):
            return cls()  # Return defaults if file doesn't exist

        with open(filepath, 'r') as f:
            data = json.load(f)
        return cls.from_dict(data)

    def __repr__(self) -> str:
        return (f"RobotParameters(wheel_dia={self.wheel_diameter*1000:.1f}mm, "
                f"wheelbase={self.wheelbase*1000:.0f}mm, "
                f"ticks/rev={self.ticks_per_revolution}, "
                f"dist/tick={self.distance_per_tick*1000:.3f}mm, "
                f"lidar_offset=({self.lidar_offset_x*1000:.1f}, {self.lidar_offset_y*1000:.1f})mm)")


class DifferentialDriveModel:
    """Differential drive kinematics and motion model

    Implements forward kinematics (encoder deltas → pose change) and
    inverse kinematics (velocity commands → wheel speeds).

    Attributes:
        params: Robot physical parameters
        motion_noise_linear: Linear motion noise std dev (m/m)
        motion_noise_angular: Angular motion noise std dev (rad/rad)
    """

    def __init__(self, params: RobotParameters = None,
                 motion_noise_linear: float = 0.02,
                 motion_noise_angular: float = 0.02):
        """Initialize motion model

        Args:
            params: Robot parameters (uses defaults if None)
            motion_noise_linear: Linear motion uncertainty (std dev as fraction of distance)
            motion_noise_angular: Angular motion uncertainty (std dev as fraction of angle)
        """
        self.params = params if params else RobotParameters()
        self.motion_noise_linear = motion_noise_linear
        self.motion_noise_angular = motion_noise_angular

    def integrate_odometry(self, pose: Pose, delta_left: int, delta_right: int,
                          dt: float) -> Pose:
        """Forward kinematics: integrate encoder deltas to update pose

        Implements differential drive kinematics:
        - Straight line motion (d_theta ≈ 0): Simple linear translation
        - Curved motion (d_theta != 0): Arc motion around ICC

        Args:
            pose: Current robot pose (x, y, theta, timestamp)
            delta_left: Left wheel encoder ticks since last update
            delta_right: Right wheel encoder ticks since last update
            dt: Time elapsed since last update (seconds)

        Returns:
            New pose after integrating motion
        """
        # Convert encoder ticks to distances
        d_left = delta_left * self.params.distance_per_tick
        d_right = delta_right * self.params.distance_per_tick

        # Calculate linear and angular displacement
        d_center = (d_left + d_right) / 2.0  # Distance traveled by robot center
        d_theta = (d_right - d_left) / self.params.wheelbase  # Change in heading (positive = left/CCW)

        # Handle two cases: straight line vs curved motion
        if abs(d_theta) < 1e-6:  # Straight line motion (threshold: ~0.0001 radians)
            # Simple linear translation in current heading direction
            dx = d_center * np.cos(pose.theta)
            dy = d_center * np.sin(pose.theta)
            new_x = pose.x + dx
            new_y = pose.y + dy
            new_theta = pose.theta  # No rotation
        else:
            # Curved motion - robot follows an arc around ICC (Instantaneous Center of Curvature)
            # ICC is located at distance R from robot center, perpendicular to heading
            R = d_center / d_theta  # Radius of curvature

            # Calculate ICC position in global frame
            icc_x = pose.x - R * np.sin(pose.theta)
            icc_y = pose.y + R * np.cos(pose.theta)

            # Rotate around ICC by d_theta
            # New position is old position rotated around ICC
            cos_dtheta = np.cos(d_theta)
            sin_dtheta = np.sin(d_theta)

            # Translate to ICC frame, rotate, translate back
            dx_icc = pose.x - icc_x
            dy_icc = pose.y - icc_y

            new_x = icc_x + (cos_dtheta * dx_icc - sin_dtheta * dy_icc)
            new_y = icc_y + (sin_dtheta * dx_icc + cos_dtheta * dy_icc)
            new_theta = self.normalize_angle(pose.theta + d_theta)

        return Pose(
            x=new_x,
            y=new_y,
            theta=new_theta,
            timestamp=pose.timestamp + dt
        )

    def predict_pose_with_noise(self, pose: Pose, delta_left: int, delta_right: int,
                                dt: float) -> Tuple[Pose, np.ndarray]:
        """Forward kinematics with motion uncertainty

        Predicts new pose and computes process noise covariance matrix.
        Used for Kalman filter-based SLAM (Phase 6).

        Args:
            pose: Current robot pose
            delta_left: Left wheel encoder ticks
            delta_right: Right wheel encoder ticks
            dt: Time elapsed (seconds)

        Returns:
            Tuple of (predicted_pose, covariance_3x3)
            Covariance matrix is [x, y, theta] uncertainty
        """
        # Get deterministic prediction
        predicted_pose = self.integrate_odometry(pose, delta_left, delta_right, dt)

        # Calculate motion distance and rotation for noise scaling
        d_left = delta_left * self.params.distance_per_tick
        d_right = delta_right * self.params.distance_per_tick
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.params.wheelbase

        # Motion noise scales with distance traveled and rotation
        # Longer motions → more uncertainty
        # Add minimum noise floor to ensure positive definite covariance
        linear_std = max(abs(d_center) * self.motion_noise_linear, 1e-6)
        angular_std = max(abs(d_theta) * self.motion_noise_angular, 1e-6)

        # Build covariance matrix (simplified model)
        # Assumes x, y uncertainty increases with distance, theta with rotation
        # More sophisticated models couple x/y/theta (Phase 6)
        covariance = np.array([
            [linear_std**2, 0.0, 0.0],
            [0.0, linear_std**2, 0.0],
            [0.0, 0.0, angular_std**2]
        ])

        return predicted_pose, covariance

    def velocities_to_wheel_speeds(self, v: float, omega: float) -> Tuple[float, float]:
        """Inverse kinematics: convert robot velocity to wheel speeds

        Given desired linear velocity (v) and angular velocity (omega),
        calculate required left and right wheel speeds.

        Args:
            v: Linear velocity in m/s (forward positive)
            omega: Angular velocity in rad/s (CCW positive)

        Returns:
            Tuple of (v_left, v_right) in m/s
        """
        # Differential drive inverse kinematics
        # v = (v_left + v_right) / 2
        # omega = (v_right - v_left) / wheelbase
        #
        # Solving for v_left and v_right:
        v_left = v - (omega * self.params.wheelbase / 2.0)
        v_right = v + (omega * self.params.wheelbase / 2.0)

        return v_left, v_right

    def wheel_speeds_to_velocities(self, v_left: float, v_right: float) -> Tuple[float, float]:
        """Forward kinematics: convert wheel speeds to robot velocity

        Given left and right wheel speeds, calculate robot's linear
        and angular velocities.

        Args:
            v_left: Left wheel speed in m/s
            v_right: Right wheel speed in m/s

        Returns:
            Tuple of (v, omega)
            v: Linear velocity in m/s
            omega: Angular velocity in rad/s
        """
        # Differential drive forward velocity kinematics
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.params.wheelbase

        return v, omega

    @staticmethod
    def normalize_angle(theta: float) -> float:
        """Normalize angle to [-π, π] range

        Args:
            theta: Angle in radians

        Returns:
            Normalized angle in [-π, π]
        """
        # Wrap to [-π, π]
        while theta > np.pi:
            theta -= 2 * np.pi
        while theta < -np.pi:
            theta += 2 * np.pi
        return theta

    def get_motion_jacobian(self, pose: Pose, delta_left: int, delta_right: int) -> np.ndarray:
        """Calculate motion model Jacobian (for EKF SLAM - Phase 6)

        Computes the Jacobian matrix of the motion model with respect to
        the current pose. Used in Extended Kalman Filter for uncertainty
        propagation.

        Args:
            pose: Current robot pose
            delta_left: Left wheel encoder ticks
            delta_right: Right wheel encoder ticks

        Returns:
            3x3 Jacobian matrix G
        """
        d_left = delta_left * self.params.distance_per_tick
        d_right = delta_right * self.params.distance_per_tick
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.params.wheelbase

        # Simplified Jacobian for small motions
        # G = ∂g/∂pose where g is the motion model
        cos_theta = np.cos(pose.theta)
        sin_theta = np.sin(pose.theta)

        if abs(d_theta) < 1e-6:
            # Straight line motion Jacobian
            G = np.array([
                [1.0, 0.0, -d_center * sin_theta],
                [0.0, 1.0,  d_center * cos_theta],
                [0.0, 0.0,  1.0]
            ])
        else:
            # Curved motion Jacobian (more complex)
            R = d_center / d_theta
            G = np.array([
                [1.0, 0.0, R * (np.cos(pose.theta + d_theta) - cos_theta)],
                [0.0, 1.0, R * (np.sin(pose.theta + d_theta) - sin_theta)],
                [0.0, 0.0, 1.0]
            ])

        return G

    def __repr__(self) -> str:
        return (f"DifferentialDriveModel({self.params}, "
                f"noise_linear={self.motion_noise_linear:.3f}, "
                f"noise_angular={self.motion_noise_angular:.3f})")
