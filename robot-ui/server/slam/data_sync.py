"""
Data Synchronization Module

Synchronizes LIDAR scans with interpolated odometry readings.

The robot's sensors operate at different frequencies:
- Odometry: 10 Hz (every 100ms)
- LIDAR: ~4 Hz (batches arrive at ~37 Hz, full scans ~4 Hz)

This module buffers sensor data and interpolates odometry to match LIDAR
scan timestamps, ensuring accurate pose information for each scan.
"""

from collections import deque
from typing import Optional, Tuple, List
import numpy as np
import time

try:
    from .sensor_data import OdomReading, LidarScan, SyncedData
except ImportError:
    from sensor_data import OdomReading, LidarScan, SyncedData


class DataSynchronizer:
    """Synchronize LIDAR scans with interpolated odometry

    This class maintains circular buffers for odometry and LIDAR data,
    and interpolates odometry readings to match LIDAR scan timestamps.

    Attributes:
        odom_buffer: Circular buffer for odometry readings
        lidar_buffer: Circular buffer for LIDAR scans
        timeout_sec: Maximum age of data before discarding (seconds)
        stats: Dictionary of synchronization statistics
    """

    def __init__(self, odom_buffer_size: int = 50, lidar_buffer_size: int = 20,
                 timeout_sec: float = 1.0):
        """Initialize data synchronizer

        Args:
            odom_buffer_size: Maximum odometry readings to buffer (default: 50 = 5s @ 10Hz)
            lidar_buffer_size: Maximum LIDAR scans to buffer (default: 20 = 5s @ 4Hz)
            timeout_sec: Maximum age of data before discarding (default: 1.0s)
        """
        self.odom_buffer: deque[OdomReading] = deque(maxlen=odom_buffer_size)
        self.lidar_buffer: deque[LidarScan] = deque(maxlen=lidar_buffer_size)
        self.timeout_sec = timeout_sec

        # Statistics
        self.stats = {
            'odom_added': 0,
            'lidar_added': 0,
            'synced_generated': 0,
            'interpolation_failed': 0,
            'odom_timeout': 0,
            'lidar_timeout': 0,
            'odom_out_of_order': 0,
        }

    def add_odometry(self, odom: OdomReading) -> None:
        """Add odometry reading to buffer

        Handles out-of-order packets by maintaining sorted order.
        Removes old data beyond timeout.

        Args:
            odom: Odometry reading to add
        """
        self.stats['odom_added'] += 1

        # Check if out of order
        if len(self.odom_buffer) > 0 and odom.timestamp < self.odom_buffer[-1].timestamp:
            self.stats['odom_out_of_order'] += 1
            # Insert in sorted position
            self._insert_sorted_odom(odom)
        else:
            # Normal case: append to end
            self.odom_buffer.append(odom)

        # Remove old data
        self._cleanup_old_data()

    def _insert_sorted_odom(self, odom: OdomReading) -> None:
        """Insert odometry reading in sorted position

        Args:
            odom: Odometry reading to insert
        """
        # Convert to list, insert, convert back
        odom_list = list(self.odom_buffer)
        inserted = False

        for i, existing_odom in enumerate(odom_list):
            if odom.timestamp < existing_odom.timestamp:
                odom_list.insert(i, odom)
                inserted = True
                break

        if not inserted:
            odom_list.append(odom)

        # Rebuild deque
        self.odom_buffer.clear()
        for o in odom_list[-self.odom_buffer.maxlen:]:
            self.odom_buffer.append(o)

    def add_lidar_scan(self, scan: LidarScan) -> None:
        """Add LIDAR scan to buffer

        Args:
            scan: LIDAR scan to add
        """
        self.stats['lidar_added'] += 1
        self.lidar_buffer.append(scan)

        # Remove old data
        self._cleanup_old_data()

    def _cleanup_old_data(self) -> None:
        """Remove data older than timeout from buffers"""
        if len(self.odom_buffer) == 0:
            return

        # Get current time (use latest odom timestamp as reference)
        current_time = self.odom_buffer[-1].timestamp

        # Remove old odometry
        while (len(self.odom_buffer) > 0 and
               (current_time - self.odom_buffer[0].timestamp) > (self.timeout_sec * 1000)):
            self.odom_buffer.popleft()
            self.stats['odom_timeout'] += 1

        # Remove old LIDAR scans
        while (len(self.lidar_buffer) > 0 and
               (current_time - self.lidar_buffer[0].timestamp) > (self.timeout_sec * 1000)):
            self.lidar_buffer.popleft()
            self.stats['lidar_timeout'] += 1

    def get_synced_data(self) -> Optional[SyncedData]:
        """Get next LIDAR scan with interpolated odometry

        Returns None if insufficient data for interpolation.

        Returns:
            SyncedData object or None if interpolation not possible
        """
        if len(self.lidar_buffer) == 0:
            return None

        # Get oldest LIDAR scan
        scan = self.lidar_buffer.popleft()

        # Find odometry readings bracketing the scan timestamp
        odom_before, odom_after = self._find_bracketing_odom(scan.timestamp)

        if odom_before is None or odom_after is None:
            self.stats['interpolation_failed'] += 1
            return None

        # Interpolate odometry at scan timestamp
        odom_interpolated = self._interpolate_odom(
            odom_before, odom_after, scan.timestamp
        )

        # Calculate interpolation weight
        weight = self._calc_weight(
            odom_before.timestamp,
            odom_after.timestamp,
            scan.timestamp
        )

        self.stats['synced_generated'] += 1

        return SyncedData(
            lidar_scan=scan,
            odom_at_scan=odom_interpolated,
            odom_before=odom_before,
            odom_after=odom_after,
            interpolation_weight=weight
        )

    def _find_bracketing_odom(
        self, timestamp: float
    ) -> Tuple[Optional[OdomReading], Optional[OdomReading]]:
        """Find odom readings immediately before and after timestamp

        Args:
            timestamp: Target timestamp in milliseconds

        Returns:
            Tuple of (odom_before, odom_after) or (None, None) if not found
        """
        before: Optional[OdomReading] = None
        after: Optional[OdomReading] = None

        for odom in self.odom_buffer:
            if odom.timestamp <= timestamp:
                before = odom
            elif odom.timestamp > timestamp:
                after = odom
                break

        return before, after

    def _interpolate_odom(
        self,
        odom_before: OdomReading,
        odom_after: OdomReading,
        target_timestamp: float
    ) -> OdomReading:
        """Linear interpolation of odometry

        Args:
            odom_before: Odometry reading before target timestamp
            odom_after: Odometry reading after target timestamp
            target_timestamp: Target timestamp in milliseconds

        Returns:
            Interpolated odometry reading
        """
        weight = self._calc_weight(
            odom_before.timestamp,
            odom_after.timestamp,
            target_timestamp
        )

        # Interpolate position (linear)
        x_mm = int(odom_before.x_mm * (1 - weight) + odom_after.x_mm * weight)
        y_mm = int(odom_before.y_mm * (1 - weight) + odom_after.y_mm * weight)

        # Interpolate angle (handle wraparound)
        theta_mrad = self._interpolate_angle(
            odom_before.theta_mrad,
            odom_after.theta_mrad,
            weight
        )

        # Interpolate velocities (linear)
        vel_left = odom_before.vel_left * (1 - weight) + odom_after.vel_left * weight
        vel_right = odom_before.vel_right * (1 - weight) + odom_after.vel_right * weight

        # Delta values are not meaningful for interpolated reading
        return OdomReading(
            timestamp=target_timestamp,
            delta_left=0,
            delta_right=0,
            x_mm=x_mm,
            y_mm=y_mm,
            theta_mrad=theta_mrad,
            vel_left=vel_left,
            vel_right=vel_right
        )

    @staticmethod
    def _calc_weight(t_before: float, t_after: float, t_target: float) -> float:
        """Calculate interpolation weight [0, 1]

        Args:
            t_before: Timestamp before target
            t_after: Timestamp after target
            t_target: Target timestamp

        Returns:
            Interpolation weight (0 = use t_before, 1 = use t_after)
        """
        if t_after == t_before:
            return 0.5

        weight = (t_target - t_before) / (t_after - t_before)

        # Clamp to [0, 1]
        return max(0.0, min(1.0, weight))

    @staticmethod
    def _interpolate_angle(angle1_mrad: int, angle2_mrad: int, weight: float) -> int:
        """Interpolate angle handling wraparound at ±π

        Args:
            angle1_mrad: First angle in milliradians
            angle2_mrad: Second angle in milliradians
            weight: Interpolation weight [0, 1]

        Returns:
            Interpolated angle in milliradians
        """
        # Convert to radians
        a1 = angle1_mrad / 1000.0
        a2 = angle2_mrad / 1000.0

        # Handle wraparound (shortest path)
        diff = a2 - a1
        if diff > np.pi:
            a2 -= 2 * np.pi
        elif diff < -np.pi:
            a2 += 2 * np.pi

        # Interpolate
        result = a1 * (1 - weight) + a2 * weight

        # Wrap to [-π, π]
        while result > np.pi:
            result -= 2 * np.pi
        while result < -np.pi:
            result += 2 * np.pi

        return int(result * 1000)

    def get_stats(self) -> dict:
        """Get buffer and synchronization statistics

        Returns:
            Dictionary with statistics:
            - Buffer sizes and capacities
            - Data added counts
            - Synchronization success/failure counts
            - Timeout and out-of-order counts
        """
        return {
            'odom_buffer_size': len(self.odom_buffer),
            'lidar_buffer_size': len(self.lidar_buffer),
            'odom_buffer_capacity': self.odom_buffer.maxlen,
            'lidar_buffer_capacity': self.lidar_buffer.maxlen,
            'odom_added': self.stats['odom_added'],
            'lidar_added': self.stats['lidar_added'],
            'synced_generated': self.stats['synced_generated'],
            'interpolation_failed': self.stats['interpolation_failed'],
            'odom_timeout': self.stats['odom_timeout'],
            'lidar_timeout': self.stats['lidar_timeout'],
            'odom_out_of_order': self.stats['odom_out_of_order'],
        }

    def get_buffer_time_span(self) -> Tuple[Optional[float], Optional[float]]:
        """Get time span covered by odometry buffer

        Returns:
            (min_timestamp, max_timestamp) in milliseconds or (None, None) if empty
        """
        if len(self.odom_buffer) == 0:
            return (None, None)

        return (self.odom_buffer[0].timestamp, self.odom_buffer[-1].timestamp)

    def reset(self) -> None:
        """Clear all buffers and reset statistics"""
        self.odom_buffer.clear()
        self.lidar_buffer.clear()

        # Reset stats
        for key in self.stats:
            self.stats[key] = 0

    def __repr__(self) -> str:
        min_t, max_t = self.get_buffer_time_span()
        time_span = (max_t - min_t) / 1000.0 if min_t and max_t else 0.0

        return (f"DataSynchronizer("
                f"odom={len(self.odom_buffer)}/{self.odom_buffer.maxlen}, "
                f"lidar={len(self.lidar_buffer)}/{self.lidar_buffer.maxlen}, "
                f"span={time_span:.1f}s, "
                f"synced={self.stats['synced_generated']})")
