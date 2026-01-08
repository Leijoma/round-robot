#!/usr/bin/env python3
import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sensor_data import OdomReading, Pose
from motion_model import RobotParameters, DifferentialDriveModel
from dead_reckoning import DeadReckoning

# Test 1: Robot parameters
params = RobotParameters()
print('Test 1: Robot Parameters')
print(f'  wheel_radius: {params.wheel_radius}')
print(f'  expected: 0.041')
print(f'  distance_per_tick: {params.distance_per_tick}')
print(f'  expected: ~0.000717')
print()

# Test 2: Curved motion
print('Test 2: Curved Motion')
model = DifferentialDriveModel()
pose = Pose(x=0.0, y=0.0, theta=0.0, timestamp=0.0)
delta_left = 50
delta_right = 150
new_pose = model.integrate_odometry(pose, delta_left, delta_right, 0.1)
print(f'  new_pose.x: {new_pose.x}')
print(f'  new_pose.y: {new_pose.y}')
print(f'  new_pose.theta: {new_pose.theta}')
print(f'  Right wheel moved more, so robot turns right (CW)')
print(f'  CW turn means negative theta')
print()

# Test 3: Angle normalization
print('Test 3: Angle Normalization')
a1 = DifferentialDriveModel.normalize_angle(3 * np.pi)
a2 = DifferentialDriveModel.normalize_angle(-3 * np.pi)
print(f'  normalize(3π={3*np.pi:.4f}): {a1:.4f}')
print(f'  expected: -π={-np.pi:.4f}')
print(f'  normalize(-3π={-3*np.pi:.4f}): {a2:.4f}')
print(f'  expected: π={np.pi:.4f}')
print()

# Test 4: Motion with noise
print('Test 4: Motion with Noise')
new_pose, cov = model.predict_pose_with_noise(pose, 100, 100, 0.1)
print(f'  covariance shape: {cov.shape}')
print(f'  covariance diagonal: [{cov[0,0]:.6f}, {cov[1,1]:.6f}, {cov[2,2]:.6f}]')
print(f'  All should be > 0')
print()

# Test 5: Path tracking
print('Test 5: Path Tracking')
dr = DeadReckoning()
for i in range(10):
    odom = OdomReading(
        timestamp=1000.0 + i * 100,
        delta_left=10, delta_right=10,
        x_mm=0, y_mm=0, theta_mrad=0,
        vel_left=0.1, vel_right=0.1
    )
    dr.update(odom)
path = dr.get_path()
print(f'  path length: {len(path)}')
print(f'  expected: 11 (initial + 10 updates)')
print(f'  BUT: first update just stores baseline, so no new pose')
print(f'  actual new poses: {len(path) - 1}')
