"""
Dead Reckoning Tracker

Tracks robot pose using only odometry (no LIDAR corrections).
Used to measure odometry drift and validate motion model.

Dead reckoning accumulates odometry over time, which causes drift due to:
- Wheel slip
- Encoder quantization
- Uneven floors
- Motion model approximations

This module provides a baseline for comparison with SLAM-corrected poses.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict
import numpy as np
import time

try:
    from .sensor_data import OdomReading, Pose
    from .motion_model import DifferentialDriveModel, RobotParameters
except ImportError:
    from sensor_data import OdomReading, Pose
    from motion_model import DifferentialDriveModel, RobotParameters


@dataclass
class DeadReckoningState:
    """Current state of dead reckoning tracker

    Attributes:
        pose: Current estimated pose
        total_distance: Total distance traveled (m)
        total_rotation: Total rotation (rad, not normalized)
        num_updates: Number of odometry updates processed
        start_time: Timestamp when tracking started
        last_odom: Last odometry reading processed
    """
    pose: Pose
    total_distance: float = 0.0
    total_rotation: float = 0.0
    num_updates: int = 0
    start_time: float = 0.0
    last_odom: Optional[OdomReading] = None

    def to_dict(self) -> Dict:
        """Convert to dictionary for JSON serialization

        Returns:
            Dictionary representation
        """
        return {
            'pose': self.pose.to_dict(),
            'total_distance': self.total_distance,
            'total_rotation': self.total_rotation,
            'total_rotation_deg': np.rad2deg(self.total_rotation),
            'num_updates': self.num_updates,
            'start_time': self.start_time,
            'elapsed_time': time.time() - self.start_time,
        }


class DeadReckoning:
    """Dead reckoning pose tracker using only odometry

    Integrates wheel encoder readings to estimate robot pose over time.
    Does not use LIDAR or any external corrections, so drift accumulates.

    Attributes:
        motion_model: Differential drive kinematics
        state: Current tracking state
        path_history: List of poses along robot's path
        max_history_size: Maximum number of poses to store
    """

    def __init__(self, motion_model: DifferentialDriveModel = None,
                 initial_pose: Pose = None,
                 max_history_size: int = 1000):
        """Initialize dead reckoning tracker

        Args:
            motion_model: Motion model for kinematics (uses defaults if None)
            initial_pose: Starting pose (defaults to origin)
            max_history_size: Maximum poses to keep in history
        """
        self.motion_model = motion_model if motion_model else DifferentialDriveModel()

        # Initialize at origin if no starting pose given
        if initial_pose is None:
            initial_pose = Pose(x=0.0, y=0.0, theta=0.0, timestamp=time.time())

        self.state = DeadReckoningState(
            pose=initial_pose,
            start_time=initial_pose.timestamp
        )

        self.path_history: List[Pose] = [initial_pose]
        self.max_history_size = max_history_size

    def update(self, odom: OdomReading) -> Pose:
        """Update pose estimate with new odometry reading

        Args:
            odom: New odometry reading

        Returns:
            Updated pose estimate
        """
        # First update - just store the reading
        if self.state.last_odom is None:
            self.state.last_odom = odom
            self.state.pose.timestamp = odom.timestamp_sec
            return self.state.pose

        # Calculate time delta
        dt = (odom.timestamp - self.state.last_odom.timestamp) / 1000.0  # ms to seconds

        # Get encoder deltas
        delta_left = odom.delta_left
        delta_right = odom.delta_right

        # Integrate motion using motion model
        new_pose = self.motion_model.integrate_odometry(
            self.state.pose,
            delta_left,
            delta_right,
            dt
        )

        # Update state
        self.state.pose = new_pose
        self.state.num_updates += 1
        self.state.last_odom = odom

        # Calculate distances for statistics
        d_left = delta_left * self.motion_model.params.distance_per_tick
        d_right = delta_right * self.motion_model.params.distance_per_tick
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.motion_model.params.wheelbase

        self.state.total_distance += abs(d_center)
        self.state.total_rotation += abs(d_theta)

        # Add to path history
        self.path_history.append(new_pose)

        # Limit history size (keep most recent)
        if len(self.path_history) > self.max_history_size:
            self.path_history.pop(0)

        return new_pose

    def get_current_pose(self) -> Pose:
        """Get current pose estimate

        Returns:
            Current pose
        """
        return self.state.pose

    def get_path(self, max_points: Optional[int] = None) -> List[Pose]:
        """Get path history

        Args:
            max_points: Maximum number of points to return (most recent)
                       If None, returns all points

        Returns:
            List of poses along path
        """
        if max_points is None or max_points >= len(self.path_history):
            return self.path_history.copy()

        # Return most recent max_points
        return self.path_history[-max_points:]

    def get_path_as_array(self, max_points: Optional[int] = None) -> np.ndarray:
        """Get path as Nx2 array of (x, y) coordinates

        Args:
            max_points: Maximum number of points to return

        Returns:
            Nx2 numpy array of (x, y) coordinates
        """
        path = self.get_path(max_points)
        if len(path) == 0:
            return np.empty((0, 2))

        return np.array([[p.x, p.y] for p in path])

    def measure_drift(self, return_pose: Pose) -> Dict:
        """Measure drift by comparing current pose to known return position

        Use this after a closed-loop path (e.g., drive in a square and return
        to start). The difference between current pose and start pose is the
        accumulated drift.

        Args:
            return_pose: Known true pose (e.g., starting position)

        Returns:
            Dictionary with drift measurements:
            - position_error: Euclidean distance error (m)
            - x_error: X position error (m)
            - y_error: Y position error (m)
            - heading_error: Heading angle error (rad)
            - drift_per_meter: Position error per meter traveled
            - drift_per_rotation: Heading error per radian rotated
        """
        current = self.state.pose

        # Position error
        dx = current.x - return_pose.x
        dy = current.y - return_pose.y
        position_error = np.sqrt(dx**2 + dy**2)

        # Heading error (normalized to [-π, π])
        heading_error = current.angle_difference(return_pose)

        # Normalized drift metrics
        drift_per_meter = position_error / self.state.total_distance if self.state.total_distance > 0 else 0.0
        drift_per_rotation = abs(heading_error) / self.state.total_rotation if self.state.total_rotation > 0 else 0.0

        return {
            'position_error': position_error,
            'x_error': dx,
            'y_error': dy,
            'heading_error': heading_error,
            'heading_error_deg': np.rad2deg(heading_error),
            'drift_per_meter': drift_per_meter,
            'drift_per_rotation': drift_per_rotation,
            'total_distance': self.state.total_distance,
            'total_rotation': self.state.total_rotation,
            'total_rotation_deg': np.rad2deg(self.state.total_rotation),
            'num_updates': self.state.num_updates,
        }

    def get_state(self) -> DeadReckoningState:
        """Get complete tracking state

        Returns:
            Current state object
        """
        return self.state

    def reset(self, initial_pose: Pose = None) -> None:
        """Reset tracker to initial state

        Args:
            initial_pose: New starting pose (defaults to origin)
        """
        if initial_pose is None:
            initial_pose = Pose(x=0.0, y=0.0, theta=0.0, timestamp=time.time())

        self.state = DeadReckoningState(
            pose=initial_pose,
            start_time=initial_pose.timestamp
        )

        self.path_history = [initial_pose]

    def export_path_for_visualization(self) -> Dict:
        """Export path in format suitable for UI visualization

        Returns:
            Dictionary with path data:
            - points: List of [x, y] coordinates
            - timestamps: List of timestamps
            - headings: List of heading angles (degrees)
            - total_distance: Total distance traveled
        """
        path = self.path_history

        return {
            'points': [[p.x, p.y] for p in path],
            'timestamps': [p.timestamp for p in path],
            'headings': [np.rad2deg(p.theta) for p in path],
            'total_distance': self.state.total_distance,
            'num_points': len(path),
        }

    def __repr__(self) -> str:
        return (f"DeadReckoning(pose={self.state.pose}, "
                f"distance={self.state.total_distance:.3f}m, "
                f"rotation={np.rad2deg(self.state.total_rotation):.1f}°, "
                f"updates={self.state.num_updates})")
