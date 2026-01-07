"""
SLAM Module for Round Robot

This package contains all SLAM-related functionality:
- sensor_data: Data classes for odometry and LIDAR readings
- data_sync: Synchronization and interpolation of sensor data
- motion_model: Differential drive kinematics (Phase 3)
- scan_matching: ICP and other scan matching algorithms (Phase 4)
- mapping: Occupancy grid and map management (Phase 5)
- slam_core: Main SLAM algorithm (Phase 6)
"""

__version__ = '0.1.0'
__author__ = 'Magnus & Claude'

from .sensor_data import (
    OdomReading,
    LidarReading,
    LidarScan,
    Pose,
    SyncedData
)

from .data_sync import DataSynchronizer
from .motion_model import RobotParameters, DifferentialDriveModel
from .dead_reckoning import DeadReckoning, DeadReckoningState
from .lidar_processing import LidarProcessor
from .scan_matcher import ICPScanMatcher

__all__ = [
    'OdomReading',
    'LidarReading',
    'LidarScan',
    'Pose',
    'SyncedData',
    'DataSynchronizer',
    'RobotParameters',
    'DifferentialDriveModel',
    'DeadReckoning',
    'DeadReckoningState',
    'LidarProcessor',
    'ICPScanMatcher',
]
