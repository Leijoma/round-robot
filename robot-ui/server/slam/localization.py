"""
Integrated Localization Module

Combines dead reckoning (odometry-only) with ICP scan matching for
drift-corrected pose estimation.

Tracks two pose estimates:
1. Dead reckoning pose: Odometry integration (accumulates drift)
2. ICP-corrected pose: Dead reckoning + scan matching corrections

The ICP-corrected pose is more accurate but may fail in featureless
environments. Dead reckoning provides a fallback.
"""

import numpy as np
from typing import Optional, Tuple, List
import time

try:
    from .sensor_data import Pose, OdomReading, LidarScan
    from .dead_reckoning import DeadReckoning
    from .motion_model import DifferentialDriveModel, RobotParameters
    from .lidar_processing import LidarProcessor
    from .scan_matcher import ICPScanMatcher
except ImportError:
    from sensor_data import Pose, OdomReading, LidarScan
    from dead_reckoning import DeadReckoning
    from motion_model import DifferentialDriveModel, RobotParameters
    from lidar_processing import LidarProcessor
    from scan_matcher import ICPScanMatcher


class IntegratedLocalizer:
    """Integrated localization using odometry and scan matching

    Maintains two pose estimates:
    - dead_reckoning_pose: Odometry integration only
    - corrected_pose: Dead reckoning + ICP corrections

    The corrected pose is updated when:
    1. A new LIDAR scan arrives
    2. ICP successfully matches it to the previous scan
    3. The match quality is acceptable

    Attributes:
        dead_reckoning: Dead reckoning tracker
        lidar_processor: LIDAR data filter
        scan_matcher: ICP scan matcher
        corrected_pose: Current ICP-corrected pose
        last_scan: Previous LIDAR scan (for matching)
        last_scan_points: Previous scan as point cloud
        icp_corrections: History of ICP corrections
    """

    def __init__(
        self,
        robot_params: Optional[RobotParameters] = None,
        initial_pose: Optional[Pose] = None
    ):
        """Initialize integrated localizer

        Args:
            robot_params: Robot physical parameters
            initial_pose: Starting pose (defaults to origin)
        """
        # Initialize components
        motion_model = DifferentialDriveModel(params=robot_params)
        self.dead_reckoning = DeadReckoning(
            motion_model=motion_model,
            initial_pose=initial_pose
        )

        self.lidar_processor = LidarProcessor(
            max_range_m=5.0,
            min_range_m=0.1,
            min_signal_strength=10,
            outlier_threshold=3.0
        )

        self.scan_matcher = ICPScanMatcher(
            max_iterations=50,
            convergence_threshold=1e-5,
            max_correspondence_distance=0.5,
            outlier_ratio=0.3
        )

        # Corrected pose starts at same position as dead reckoning
        if initial_pose is None:
            initial_pose = Pose(x=0.0, y=0.0, theta=0.0, timestamp=time.time())
        self.corrected_pose = initial_pose

        # Previous scan for matching
        self.last_scan: Optional[LidarScan] = None
        self.last_scan_points: Optional[np.ndarray] = None

        # Statistics
        self.icp_corrections: List[Tuple[float, float, float]] = []
        self.num_scans_processed = 0
        self.num_icp_successes = 0
        self.num_icp_failures = 0
        self.total_correction_distance = 0.0
        self.total_correction_rotation = 0.0

    def update_odometry(self, odom: OdomReading) -> Pose:
        """Update with new odometry reading

        Args:
            odom: Odometry reading

        Returns:
            Updated dead reckoning pose
        """
        return self.dead_reckoning.update(odom)

    def update_scan(self, scan: LidarScan) -> Tuple[Pose, bool, dict]:
        """Update with new LIDAR scan

        Processes scan and attempts ICP matching. If successful, corrects
        the pose estimate.

        Args:
            scan: Raw LIDAR scan

        Returns:
            Tuple of (corrected_pose, icp_success, match_info):
            - corrected_pose: Updated pose estimate
            - icp_success: True if ICP matched successfully
            - match_info: Dictionary with ICP statistics
        """
        self.num_scans_processed += 1

        # Filter scan
        filtered_scan = self.lidar_processor.filter_scan(scan)

        # Convert to point cloud (robot frame)
        current_points, _ = self.lidar_processor.polar_to_cartesian(filtered_scan)

        # Check if we have enough points
        if len(current_points) < 20:
            match_info = {
                'success': False,
                'reason': 'insufficient_points',
                'num_points': len(current_points)
            }
            self.num_icp_failures += 1
            return self.corrected_pose, False, match_info

        # If this is the first scan, just store it
        if self.last_scan is None or self.last_scan_points is None:
            self.last_scan = filtered_scan
            self.last_scan_points = current_points
            match_info = {
                'success': False,
                'reason': 'first_scan',
                'num_points': len(current_points)
            }
            return self.corrected_pose, False, match_info

        # Get dead reckoning estimate for initial guess
        dr_pose = self.dead_reckoning.get_current_pose()

        # Compute dead reckoning motion since last scan
        # (Use as initial guess for ICP)
        dx_dr = dr_pose.x - self.corrected_pose.x
        dy_dr = dr_pose.y - self.corrected_pose.y
        dtheta_dr = dr_pose.theta - self.corrected_pose.theta

        # Normalize angle difference
        while dtheta_dr > np.pi:
            dtheta_dr -= 2 * np.pi
        while dtheta_dr < -np.pi:
            dtheta_dr += 2 * np.pi

        initial_guess = (dx_dr, dy_dr, dtheta_dr)

        # Run ICP
        transform, error, converged = self.scan_matcher.match(
            current_points,
            self.last_scan_points,
            initial_transform=initial_guess
        )

        # Check match quality
        quality = self.scan_matcher.estimate_quality(
            current_points,
            self.last_scan_points,
            transform
        )

        # Decide if ICP result is acceptable
        icp_success = (
            converged and
            quality['correspondence_ratio'] > 0.4 and
            quality['rms_error'] < 0.1
        )

        if icp_success:
            # Apply ICP correction to pose
            dx_icp, dy_icp, dtheta_icp = transform

            # Update corrected pose
            # Transform is in corrected pose frame, so add to it
            cos_theta = np.cos(self.corrected_pose.theta)
            sin_theta = np.sin(self.corrected_pose.theta)

            self.corrected_pose = Pose(
                x=self.corrected_pose.x + dx_icp * cos_theta - dy_icp * sin_theta,
                y=self.corrected_pose.y + dx_icp * sin_theta + dy_icp * cos_theta,
                theta=self.corrected_pose.theta + dtheta_icp,
                timestamp=scan.timestamp / 1000.0  # Convert ms to seconds
            )

            # Normalize angle
            self.corrected_pose.theta = np.arctan2(
                np.sin(self.corrected_pose.theta),
                np.cos(self.corrected_pose.theta)
            )

            # Track corrections
            self.icp_corrections.append(transform)
            self.num_icp_successes += 1
            self.total_correction_distance += np.sqrt(dx_icp**2 + dy_icp**2)
            self.total_correction_rotation += abs(dtheta_icp)

            # Limit correction history
            if len(self.icp_corrections) > 100:
                self.icp_corrections.pop(0)

            match_info = {
                'success': True,
                'transform': transform,
                'error': error,
                'quality': quality,
                'initial_guess': initial_guess
            }
        else:
            # ICP failed - use dead reckoning estimate
            self.corrected_pose = Pose(
                x=dr_pose.x,
                y=dr_pose.y,
                theta=dr_pose.theta,
                timestamp=scan.timestamp / 1000.0
            )

            self.num_icp_failures += 1

            match_info = {
                'success': False,
                'reason': 'quality_check_failed',
                'converged': converged,
                'quality': quality,
                'initial_guess': initial_guess
            }

        # Store scan for next iteration
        self.last_scan = filtered_scan
        self.last_scan_points = current_points

        return self.corrected_pose, icp_success, match_info

    def get_dead_reckoning_pose(self) -> Pose:
        """Get current dead reckoning (odometry-only) pose

        Returns:
            Dead reckoning pose
        """
        return self.dead_reckoning.get_current_pose()

    def get_corrected_pose(self) -> Pose:
        """Get current ICP-corrected pose

        Returns:
            Corrected pose estimate
        """
        return self.corrected_pose

    def get_pose_drift(self) -> dict:
        """Measure drift between dead reckoning and corrected pose

        Returns:
            Dictionary with drift measurements
        """
        dr_pose = self.dead_reckoning.get_current_pose()

        dx = dr_pose.x - self.corrected_pose.x
        dy = dr_pose.y - self.corrected_pose.y
        distance_drift = np.sqrt(dx**2 + dy**2)

        dtheta = dr_pose.theta - self.corrected_pose.theta
        # Normalize
        while dtheta > np.pi:
            dtheta -= 2 * np.pi
        while dtheta < -np.pi:
            dtheta += 2 * np.pi

        return {
            'distance_drift': distance_drift,
            'x_drift': dx,
            'y_drift': dy,
            'heading_drift': dtheta,
            'heading_drift_deg': np.rad2deg(dtheta),
        }

    def get_statistics(self) -> dict:
        """Get localization statistics

        Returns:
            Dictionary with statistics
        """
        dr_state = self.dead_reckoning.get_state()
        drift = self.get_pose_drift()

        success_rate = (
            self.num_icp_successes / self.num_scans_processed
            if self.num_scans_processed > 0 else 0.0
        )

        avg_correction_dist = (
            self.total_correction_distance / self.num_icp_successes
            if self.num_icp_successes > 0 else 0.0
        )

        avg_correction_rot = (
            self.total_correction_rotation / self.num_icp_successes
            if self.num_icp_successes > 0 else 0.0
        )

        return {
            'scans_processed': self.num_scans_processed,
            'icp_successes': self.num_icp_successes,
            'icp_failures': self.num_icp_failures,
            'icp_success_rate': success_rate,
            'avg_correction_distance': avg_correction_dist,
            'avg_correction_rotation': avg_correction_rot,
            'avg_correction_rotation_deg': np.rad2deg(avg_correction_rot),
            'total_distance_traveled': dr_state.total_distance,
            'total_rotation': np.rad2deg(dr_state.total_rotation),
            'current_drift': drift,
        }

    def reset(self, initial_pose: Optional[Pose] = None):
        """Reset localizer to initial state

        Args:
            initial_pose: New starting pose
        """
        if initial_pose is None:
            initial_pose = Pose(x=0.0, y=0.0, theta=0.0, timestamp=time.time())

        self.dead_reckoning.reset(initial_pose)
        self.corrected_pose = initial_pose
        self.last_scan = None
        self.last_scan_points = None
        self.icp_corrections = []
        self.num_scans_processed = 0
        self.num_icp_successes = 0
        self.num_icp_failures = 0
        self.total_correction_distance = 0.0
        self.total_correction_rotation = 0.0

    def __repr__(self) -> str:
        stats = self.get_statistics()
        return (f"IntegratedLocalizer(scans={stats['scans_processed']}, "
                f"icp_success_rate={stats['icp_success_rate']:.1%}, "
                f"drift={stats['current_drift']['distance_drift']:.3f}m)")
