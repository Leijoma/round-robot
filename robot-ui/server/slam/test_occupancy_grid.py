"""
Unit Tests for Occupancy Grid Module

Tests coordinate transformations, Bresenham ray tracing, log-odds updates,
and LIDAR scan integration.
"""

import numpy as np
import math
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from occupancy_grid import OccupancyGrid
from sensor_data import Pose, LidarScan, LidarReading


def test_initialization():
    """Test grid initialization with default parameters"""
    grid = OccupancyGrid(width=100, height=100, resolution=0.05)

    assert grid.width == 100
    assert grid.height == 100
    assert grid.resolution == 0.05
    assert grid.log_odds.shape == (100, 100)
    assert np.all(grid.log_odds == 0.0), "All cells should start as unknown"

    print("✓ Initialization test passed")


def test_world_to_grid_conversion():
    """Test world coordinates to grid indices conversion"""
    grid = OccupancyGrid(width=100, height=100, resolution=0.05)

    # Test origin (0, 0) -> center of grid
    gx, gy = grid.world_to_grid(0.0, 0.0)
    assert gx == 50 and gy == 50, f"Origin should map to center: got ({gx}, {gy})"

    # Test 1m right (positive X)
    gx, gy = grid.world_to_grid(1.0, 0.0)
    assert gx == 70, f"1m right should be 20 cells: got gx={gx}"  # 1.0m / 0.05m/cell = 20 cells

    # Test 1m up (positive Y)
    gx, gy = grid.world_to_grid(0.0, 1.0)
    assert gy == 30, f"1m up should be 20 cells up: got gy={gy}"  # Y inverted

    # Test negative values
    gx, gy = grid.world_to_grid(-1.0, -1.0)
    assert gx == 30 and gy == 70, f"(-1, -1) should map to (30, 70): got ({gx}, {gy})"

    print("✓ World to grid conversion test passed")


def test_grid_to_world_conversion():
    """Test grid indices to world coordinates conversion"""
    grid = OccupancyGrid(width=100, height=100, resolution=0.05)

    # Test center
    x, y = grid.grid_to_world(50, 50)
    assert abs(x) < 1e-6 and abs(y) < 1e-6, f"Center should be origin: got ({x}, {y})"

    # Test round trip
    x_orig, y_orig = 1.234, -0.567
    gx, gy = grid.world_to_grid(x_orig, y_orig)
    x_back, y_back = grid.grid_to_world(gx, gy)

    assert abs(x_back - x_orig) < grid.resolution, "X round trip failed"
    assert abs(y_back - y_orig) < grid.resolution, "Y round trip failed"

    print("✓ Grid to world conversion test passed")


def test_is_valid_cell():
    """Test cell bounds checking"""
    grid = OccupancyGrid(width=100, height=100)

    assert grid.is_valid_cell(0, 0), "Top-left should be valid"
    assert grid.is_valid_cell(99, 99), "Bottom-right should be valid"
    assert grid.is_valid_cell(50, 50), "Center should be valid"

    assert not grid.is_valid_cell(-1, 50), "Negative X should be invalid"
    assert not grid.is_valid_cell(50, -1), "Negative Y should be invalid"
    assert not grid.is_valid_cell(100, 50), "X=width should be invalid"
    assert not grid.is_valid_cell(50, 100), "Y=height should be invalid"

    print("✓ Cell bounds checking test passed")


def test_bresenham_ray_tracing():
    """Test Bresenham line algorithm"""
    grid = OccupancyGrid()

    # Test horizontal line
    cells = grid.bresenham_ray(0, 0, 10, 0)
    assert len(cells) == 11, f"Horizontal line should have 11 cells: got {len(cells)}"
    assert cells[0] == (0, 0), "Should start at origin"
    assert cells[-1] == (10, 0), "Should end at (10, 0)"

    # Test vertical line
    cells = grid.bresenham_ray(0, 0, 0, 10)
    assert len(cells) == 11, f"Vertical line should have 11 cells: got {len(cells)}"
    assert cells[0] == (0, 0)
    assert cells[-1] == (0, 10)

    # Test diagonal line
    cells = grid.bresenham_ray(0, 0, 10, 10)
    assert len(cells) == 11, f"Diagonal line should have 11 cells: got {len(cells)}"
    assert cells[0] == (0, 0)
    assert cells[-1] == (10, 10)

    # Test that all cells are connected (no gaps)
    for i in range(len(cells) - 1):
        x0, y0 = cells[i]
        x1, y1 = cells[i + 1]
        dist = abs(x1 - x0) + abs(y1 - y0)
        assert dist <= 2, f"Cells should be adjacent: ({x0},{y0}) to ({x1},{y1})"

    print("✓ Bresenham ray tracing test passed")


def test_log_odds_updates():
    """Test log-odds probability updates"""
    grid = OccupancyGrid()

    # Test occupied cell update
    gx, gy = 50, 50
    initial_log_odds = grid.log_odds[gy, gx]
    assert initial_log_odds == 0.0, "Should start at 0 (unknown)"

    # Mark as occupied
    grid.log_odds[gy, gx] += grid.l_occ
    prob = 1.0 / (1.0 + np.exp(-grid.log_odds[gy, gx]))
    assert prob > 0.5, f"Occupied cell should be >50% probability: got {prob:.3f}"

    # Test free cell update
    gx, gy = 51, 51
    grid.log_odds[gy, gx] += grid.l_free
    prob = 1.0 / (1.0 + np.exp(-grid.log_odds[gy, gx]))
    assert prob < 0.5, f"Free cell should be <50% probability: got {prob:.3f}"

    # Test multiple updates accumulate
    gx, gy = 52, 52
    for _ in range(5):
        grid.log_odds[gy, gx] += grid.l_occ
    prob = 1.0 / (1.0 + np.exp(-grid.log_odds[gy, gx]))
    assert prob > 0.9, f"Multiple occupied updates should increase confidence: got {prob:.3f}"

    # Test clamping
    grid.log_odds[gy, gx] = grid.l_max + 10  # Way beyond max
    grid.log_odds[gy, gx] = np.clip(grid.log_odds[gy, gx], grid.l_min, grid.l_max)
    assert grid.log_odds[gy, gx] == grid.l_max, "Should clamp to maximum"

    print("✓ Log-odds updates test passed")


def test_get_probability_grid():
    """Test log-odds to probability conversion"""
    grid = OccupancyGrid()

    # Set some test values
    grid.log_odds[50, 50] = grid.l_max   # Very occupied
    grid.log_odds[51, 51] = 0.0          # Unknown
    grid.log_odds[52, 52] = grid.l_min   # Very free

    prob_grid = grid.get_probability_grid()

    assert prob_grid[50, 50] > 0.9, "Max log-odds should give high probability"
    assert 0.45 < prob_grid[51, 51] < 0.55, "Zero log-odds should give 50% probability"
    assert prob_grid[52, 52] < 0.1, "Min log-odds should give low probability"

    print("✓ Probability grid conversion test passed")


def test_get_display_grid():
    """Test discretization to display values"""
    grid = OccupancyGrid()

    # Set up test probabilities
    # Free: p < 0.3
    grid.log_odds[50, 50] = -2.0  # p ≈ 0.12 (free)

    # Unknown: 0.3 <= p <= 0.7
    grid.log_odds[51, 51] = 0.0  # p = 0.5 (unknown)

    # Occupied: p > 0.7
    grid.log_odds[52, 52] = 2.0  # p ≈ 0.88 (occupied)

    display_grid = grid.get_display_grid()

    assert display_grid[50, 50] == 0, "Should be marked as free"
    assert display_grid[51, 51] == 1, "Should be marked as unknown"
    assert display_grid[52, 52] == 2, "Should be marked as occupied"

    print("✓ Display grid discretization test passed")


def test_scan_integration():
    """Test updating grid from LIDAR scan"""
    grid = OccupancyGrid(width=100, height=100, resolution=0.05)

    # Robot at origin, facing right (0 degrees)
    robot_pose = Pose(x=0.0, y=0.0, theta=0.0, timestamp=0.0)

    # Create LIDAR scan with wall at 2m directly ahead (0 degrees)
    readings = [
        LidarReading(angle_deg=0.0, distance_mm=2000, signal_strength=200, valid=True)
    ]
    scan = LidarScan(timestamp=0.0, readings=readings, rpm=220)

    # Update grid
    grid.update_from_scan(robot_pose, scan)

    # Check that endpoint is marked as occupied
    hit_gx, hit_gy = grid.world_to_grid(2.0, 0.0)  # 2m ahead at x=2.0, y=0.0
    prob = grid.get_probability_grid()

    # Endpoint should be more likely occupied
    assert prob[hit_gy, hit_gx] > 0.5, f"Hit point should be occupied: p={prob[hit_gy, hit_gx]:.3f}"

    # Cells along ray should be more likely free
    mid_gx, mid_gy = grid.world_to_grid(1.0, 0.0)  # 1m ahead (midpoint)
    assert prob[mid_gy, mid_gx] < 0.5, f"Midpoint should be free: p={prob[mid_gy, mid_gx]:.3f}"

    assert grid.scan_count == 1, "Should have processed 1 scan"
    assert grid.update_count >= 1, "Should have made at least 1 update"

    print("✓ LIDAR scan integration test passed")


def test_multiple_scans():
    """Test accumulation of multiple scans"""
    grid = OccupancyGrid()
    robot_pose = Pose(x=0.0, y=0.0, theta=0.0, timestamp=0.0)

    # Simulate multiple scans of the same obstacle
    readings = [LidarReading(angle_deg=0.0, distance_mm=2000, signal_strength=200, valid=True)]

    for i in range(5):
        scan = LidarScan(timestamp=float(i), readings=readings, rpm=220)
        grid.update_from_scan(robot_pose, scan)

    # After multiple scans, confidence should be higher
    hit_gx, hit_gy = grid.world_to_grid(2.0, 0.0)
    prob = grid.get_probability_grid()

    assert prob[hit_gy, hit_gx] > 0.8, f"Multiple scans should increase confidence: p={prob[hit_gy, hit_gx]:.3f}"
    assert grid.scan_count == 5, f"Should have processed 5 scans: got {grid.scan_count}"

    print("✓ Multiple scans accumulation test passed")


def test_clear():
    """Test grid clearing"""
    grid = OccupancyGrid()

    # Add some data
    grid.log_odds[50, 50] = 2.0
    grid.scan_count = 10
    grid.update_count = 100

    # Clear
    grid.clear()

    assert np.all(grid.log_odds == 0.0), "All cells should be reset to unknown"
    assert grid.scan_count == 0, "Scan count should be reset"
    assert grid.update_count == 0, "Update count should be reset"

    print("✓ Grid clearing test passed")


def test_serialization():
    """Test JSON serialization for UI"""
    grid = OccupancyGrid(width=100, height=100, resolution=0.05)
    robot_pose = Pose(x=1.2, y=0.5, theta=0.78, timestamp=1.0)

    # Add some data
    grid.log_odds[50, 50] = 2.0  # Occupied
    grid.scan_count = 5

    # Serialize
    data = grid.serialize_for_ui(robot_pose)

    assert data['width'] == 100
    assert data['height'] == 100
    assert data['resolution'] == 0.05
    assert isinstance(data['grid'], list), "Grid should be a list"
    assert len(data['grid']) == 100, "Grid should have 100 rows"
    assert len(data['grid'][0]) == 100, "Each row should have 100 cells"

    assert data['robot_pose'] is not None
    assert data['robot_pose']['x'] == 1.2
    assert data['robot_pose']['y'] == 0.5
    assert data['robot_pose']['theta'] == 0.78

    assert 'stats' in data
    assert data['stats']['scans'] == 5

    print("✓ Serialization test passed")


def test_get_stats():
    """Test statistics gathering"""
    grid = OccupancyGrid()

    # Set up grid with known values
    grid.log_odds[50:60, 50:60] = 2.0  # 10x10 occupied region
    grid.log_odds[40:50, 40:50] = -2.0  # 10x10 free region
    grid.scan_count = 10
    grid.update_count = 250

    stats = grid.get_stats()

    assert stats['total_cells'] == 10000, "Total cells should be width * height"
    assert stats['occupied_cells'] == 100, f"Should have 100 occupied cells: got {stats['occupied_cells']}"
    assert stats['free_cells'] == 100, f"Should have 100 free cells: got {stats['free_cells']}"
    assert stats['unknown_cells'] == 9800, f"Remaining should be unknown: got {stats['unknown_cells']}"
    assert stats['scans_processed'] == 10
    assert stats['updates_total'] == 250

    print("✓ Statistics gathering test passed")


def run_all_tests():
    """Run all unit tests"""
    print("Running Occupancy Grid Unit Tests...")
    print("=" * 60)

    test_initialization()
    test_world_to_grid_conversion()
    test_grid_to_world_conversion()
    test_is_valid_cell()
    test_bresenham_ray_tracing()
    test_log_odds_updates()
    test_get_probability_grid()
    test_get_display_grid()
    test_scan_integration()
    test_multiple_scans()
    test_clear()
    test_serialization()
    test_get_stats()

    print("=" * 60)
    print("✅ All tests passed!")


if __name__ == '__main__':
    run_all_tests()
