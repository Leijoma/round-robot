"""
Scan Matching Module - Iterative Closest Point (ICP) Algorithm

Implements ICP for aligning LIDAR scans and correcting odometry drift.
Uses point-to-point ICP with outlier rejection and correspondence filtering.
"""

import numpy as np
from scipy.spatial import cKDTree
from typing import Tuple, Optional
import time

try:
    from .sensor_data import Pose
except ImportError:
    from sensor_data import Pose


class ICPScanMatcher:
    """Iterative Closest Point scan matching for LIDAR scans

    Estimates rigid 2D transformation (dx, dy, dtheta) between two point clouds:
        target = T * source

    where T is the transformation that best aligns source to target.

    Algorithm:
    1. Find nearest neighbor correspondences (source → target)
    2. Filter outliers based on distance and ratio
    3. Estimate transformation using SVD
    4. Apply transformation to source
    5. Repeat until convergence or max iterations

    Attributes:
        max_iterations: Maximum ICP iterations
        convergence_threshold: Error change threshold for convergence (meters)
        max_correspondence_distance: Maximum distance for valid correspondence
        outlier_ratio: Fraction of worst correspondences to reject
    """

    def __init__(
        self,
        max_iterations: int = 50,
        convergence_threshold: float = 1e-5,
        max_correspondence_distance: float = 0.5,
        outlier_ratio: float = 0.3
    ):
        """Initialize ICP matcher

        Args:
            max_iterations: Maximum number of ICP iterations
            convergence_threshold: Error change for convergence (meters)
            max_correspondence_distance: Max distance for correspondences (meters)
            outlier_ratio: Fraction of correspondences to reject (0-1)
        """
        self.max_iterations = max_iterations
        self.convergence_threshold = convergence_threshold
        self.max_correspondence_distance = max_correspondence_distance
        self.outlier_ratio = outlier_ratio

    def match(
        self,
        source_points: np.ndarray,
        target_points: np.ndarray,
        initial_transform: Optional[Tuple[float, float, float]] = None
    ) -> Tuple[Tuple[float, float, float], float, bool]:
        """Match source scan to target scan using ICP

        Args:
            source_points: Nx2 array (source scan in robot frame)
            target_points: Mx2 array (target/reference scan)
            initial_transform: Initial guess (dx, dy, dtheta) or None for identity

        Returns:
            Tuple of (transform, rms_error, converged):
            - transform: (dx, dy, dtheta) in meters and radians
            - rms_error: RMS correspondence error
            - converged: True if converged before max iterations
        """
        # Check minimum point requirements
        if len(source_points) < 10 or len(target_points) < 10:
            return (0.0, 0.0, 0.0), float('inf'), False

        # Initialize transform
        if initial_transform is None:
            dx, dy, dtheta = 0.0, 0.0, 0.0
        else:
            dx, dy, dtheta = initial_transform

        # Working copy of source points
        transformed_source = self._apply_transform(
            source_points, dx, dy, dtheta
        )

        # Build KD-tree for target points (for fast nearest neighbor search)
        target_tree = cKDTree(target_points)

        prev_error = float('inf')

        for iteration in range(self.max_iterations):
            # Step 1: Find correspondences (nearest neighbors)
            distances, indices = target_tree.query(transformed_source)

            # Step 2: Filter outliers by distance
            valid_mask = distances < self.max_correspondence_distance

            if np.sum(valid_mask) < 10:
                # Too few correspondences for reliable transform
                return (dx, dy, dtheta), prev_error, False

            # Step 3: Reject worst correspondences (outlier_ratio)
            num_keep = int(len(distances) * (1 - self.outlier_ratio))
            sorted_indices = np.argsort(distances)
            keep_indices = sorted_indices[:num_keep]

            valid_mask = np.zeros(len(distances), dtype=bool)
            valid_mask[keep_indices] = True

            # Extract valid correspondences
            # Use ORIGINAL source points, not transformed
            source_matched = source_points[valid_mask]
            target_matched = target_points[indices[valid_mask]]

            # Compute mean error
            current_error = np.mean(distances[valid_mask])

            # Step 4: Check convergence
            error_change = abs(prev_error - current_error)
            if error_change < self.convergence_threshold:
                # Converged!
                return (dx, dy, dtheta), current_error, True

            prev_error = current_error

            # Step 5: Estimate transformation from correspondences
            # This computes the transform from ORIGINAL source to target
            new_transform = self._estimate_transform(
                source_matched, target_matched
            )

            # Step 6: Use the new transform directly
            dx, dy, dtheta = new_transform

            # Normalize angle to [-π, π]
            dtheta = np.arctan2(np.sin(dtheta), np.cos(dtheta))

            # Apply updated transform to source
            transformed_source = self._apply_transform(
                source_points, dx, dy, dtheta
            )

        # Max iterations reached without convergence
        return (dx, dy, dtheta), prev_error, False

    def _apply_transform(
        self,
        points: np.ndarray,
        dx: float,
        dy: float,
        dtheta: float
    ) -> np.ndarray:
        """Apply 2D rigid transformation to points

        Applies rotation followed by translation:
            p' = R(dtheta) * p + [dx, dy]

        Args:
            points: Nx2 array of points
            dx: Translation in X (meters)
            dy: Translation in Y (meters)
            dtheta: Rotation angle (radians)

        Returns:
            Transformed Nx2 array
        """
        if len(points) == 0:
            return points

        cos_th = np.cos(dtheta)
        sin_th = np.sin(dtheta)

        R = np.array([
            [cos_th, -sin_th],
            [sin_th,  cos_th]
        ])

        return (R @ points.T).T + np.array([dx, dy])

    def _estimate_transform(
        self,
        source: np.ndarray,
        target: np.ndarray
    ) -> Tuple[float, float, float]:
        """Estimate transformation using SVD method

        Computes optimal rigid transformation (rotation + translation)
        that minimizes sum of squared distances between corresponding points.

        Method: SVD-based closed-form solution (Horn, 1987)

        Args:
            source: Nx2 matched points from source
            target: Nx2 matched points from target

        Returns:
            (dx, dy, dtheta) transformation
        """
        if len(source) < 3 or len(target) < 3:
            return 0.0, 0.0, 0.0

        # Compute centroids
        source_centroid = np.mean(source, axis=0)
        target_centroid = np.mean(target, axis=0)

        # Center the points
        source_centered = source - source_centroid
        target_centered = target - target_centroid

        # Compute cross-covariance matrix H = source^T * target
        H = source_centered.T @ target_centered

        # Singular Value Decomposition
        U, S, Vt = np.linalg.svd(H)

        # Compute optimal rotation R = V * U^T
        R = Vt.T @ U.T

        # Handle reflection case (det(R) = -1)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        # Extract rotation angle
        dtheta = np.arctan2(R[1, 0], R[0, 0])

        # Compute translation t = target_centroid - R * source_centroid
        t = target_centroid - R @ source_centroid
        dx, dy = t

        return dx, dy, dtheta

    def estimate_quality(
        self,
        source_points: np.ndarray,
        target_points: np.ndarray,
        transform: Tuple[float, float, float]
    ) -> dict:
        """Estimate match quality metrics

        Applies transform to source and measures correspondence errors.

        Args:
            source_points: Nx2 source points
            target_points: Mx2 target points
            transform: (dx, dy, dtheta) transformation

        Returns:
            Dictionary with quality metrics:
            - rms_error: Root mean squared error
            - mean_error: Mean correspondence distance
            - median_error: Median correspondence distance
            - max_error: Maximum correspondence distance
            - num_correspondences: Number of valid correspondences
            - correspondence_ratio: Fraction of source points matched
        """
        dx, dy, dtheta = transform

        # Apply transform
        transformed = self._apply_transform(source_points, dx, dy, dtheta)

        # Find correspondences
        target_tree = cKDTree(target_points)
        distances, indices = target_tree.query(transformed)

        # Filter by maximum correspondence distance
        valid_mask = distances < self.max_correspondence_distance
        valid_distances = distances[valid_mask]

        if len(valid_distances) == 0:
            return {
                'rms_error': float('inf'),
                'mean_error': float('inf'),
                'median_error': float('inf'),
                'max_error': float('inf'),
                'num_correspondences': 0,
                'correspondence_ratio': 0.0
            }

        return {
            'rms_error': float(np.sqrt(np.mean(valid_distances**2))),
            'mean_error': float(np.mean(valid_distances)),
            'median_error': float(np.median(valid_distances)),
            'max_error': float(np.max(valid_distances)),
            'num_correspondences': int(len(valid_distances)),
            'correspondence_ratio': float(len(valid_distances) / len(source_points))
        }

    def match_with_timing(
        self,
        source_points: np.ndarray,
        target_points: np.ndarray,
        initial_transform: Optional[Tuple[float, float, float]] = None
    ) -> Tuple[Tuple[float, float, float], float, bool, float]:
        """Match with timing information

        Same as match() but also returns execution time.

        Returns:
            Tuple of (transform, rms_error, converged, time_ms)
        """
        start_time = time.time()
        transform, error, converged = self.match(
            source_points, target_points, initial_transform
        )
        elapsed_ms = (time.time() - start_time) * 1000.0

        return transform, error, converged, elapsed_ms

    def __repr__(self) -> str:
        return (f"ICPScanMatcher(max_iter={self.max_iterations}, "
                f"conv_thresh={self.convergence_threshold}m, "
                f"max_corresp_dist={self.max_correspondence_distance}m, "
                f"outlier_ratio={self.outlier_ratio})")
