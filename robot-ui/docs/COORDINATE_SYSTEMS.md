# Coordinate Systems and Reference Frames

This document defines the coordinate systems, transformations, and conventions used throughout the SLAM system.

## Table of Contents

1. [World Frame (Global/Map Frame)](#world-frame)
2. [Robot Frame (Body Frame)](#robot-frame)
3. [Grid Frame (Occupancy Grid)](#grid-frame)
4. [Arduino Frame (Encoder/Motor)](#arduino-frame)
5. [LIDAR Frame](#lidar-frame)
6. [Transformations](#transformations)
7. [Visualization Conventions](#visualization-conventions)

---

## World Frame (Global/Map Frame)

The **world frame** is the global reference frame for all localization and mapping. It's established when the robot first starts up.

### Axis Definitions

```
      +Y (heading 90°)
       ↑
       |
       |
       |
-X ←---●---→ +X  (heading 0°)
(-90°) |
       |
       |
      -Y (heading 180°/-180°)
```

- **Origin:** Robot's starting position (typically near room corner)
- **X-axis:** Positive to the right (East), negative to the left (West)
- **Y-axis:** Positive upward/forward (North), negative downward/backward (South)
- **Heading (θ):**
  - **0° = +X direction** (rightward on visualization)
  - **90° = +Y direction** (upward on visualization)
  - **180°/-180° = -X direction** (leftward)
  - **-90°/270° = -Y direction** (downward)
- **Rotation Direction:** Counter-clockwise positive (CCW+)
- **Units:** Meters (m) for position, radians (rad) for angles

### Key Properties

- **Fixed:** Never moves once established at startup
- **Right-handed:** Follows standard mathematical convention
- **Persistent:** All historical poses and map features referenced to this frame

### Implementation

```python
# motion_model.py (line 186-187)
dx = d_center * np.cos(pose.theta)  # X displacement
dy = d_center * np.sin(pose.theta)  # Y displacement
```

**Heading examples:**
- `theta = 0.0` → Movement in +X direction (cos(0)=1, sin(0)=0)
- `theta = π/2` → Movement in +Y direction (cos(π/2)=0, sin(π/2)=1)
- `theta = π` → Movement in -X direction (cos(π)=-1, sin(π)=0)

---

## Robot Frame (Body Frame)

The **robot frame** is attached to the robot's body and moves with it.

### Axis Definitions

```
      +X (forward)
       ↑
       |
       |
 +Y ←--●  (robot center)
(left)
```

- **Origin:** Center point between the two drive wheels
- **X-axis:** Forward direction of robot (straight ahead)
- **Y-axis:** Left side of robot (perpendicular to forward)
- **Heading:** Always 0° in own frame (robot always "faces" +X)
- **Units:** Meters (m) for measurements

### Key Properties

- **Mobile:** Moves with the robot
- **LIDAR measurements:** Initially expressed in robot frame
- **IMU data:** If available, expressed in robot frame
- **Coordinate to world:** Requires robot's world-frame pose

### Physical Dimensions

```python
# Calibrated robot parameters (project_context.md)
wheelbase = 244 mm  # Distance between wheel centers
wheel_diameter = 79 mm
robot_width ≈ 244 mm  # Wheelbase
robot_length ≈ 200 mm  # Estimate (not critical for point-robot model)
```

---

## Grid Frame (Occupancy Grid)

The **grid frame** is a discretized representation of the world frame for occupancy mapping.

### Structure

```
Grid Index [0, 0]         Grid Index [139, 0]
(world: -3.5m, +3.5m)     (world: +3.5m, +3.5m)
         ●------------------------●
         |                        |
         |                        |
         |          ●             |  ← Center [70, 70]
         |      (origin marker)   |     (world: 0, 0)
         |                        |
         |                        |
         ●------------------------●
Grid Index [0, 139]       Grid Index [139, 139]
(world: -3.5m, -3.5m)     (world: +3.5m, -3.5m)
```

### Parameters

- **Dimensions:** 140 × 140 cells
- **Resolution:** 0.05 m/cell (5 cm per cell)
- **Coverage:** 7m × 7m total area
- **Origin:** Configurable via `origin_offset_x` and `origin_offset_y`

### Grid Indexing

- **Grid X (gx):** 0 (left) to 139 (right)
- **Grid Y (gy):** 0 (top) to 139 (bottom)
- **Note:** Grid Y increases downward (image convention), opposite of world Y

### Origin Offset Configurations

#### Centered (Default: offset = 0, 0)

```
World origin at grid center [70, 70]
Coverage: X: -3.5m to +3.5m, Y: -3.5m to +3.5m
Symmetric in all directions
```

#### Corner Start (Current: offset = -2.5, -2.5)

```
World origin at grid [20, 120]
Coverage: X: -2.5m to +4.5m, Y: -2.5m to +4.5m
Optimized for starting in room corner:
  - 4.5m forward space (+Y)
  - 4.5m right space (+X)
  - 2.5m backward space (-Y)
  - 2.5m left space (-X)
```

### Cell Values

- **0:** Free space (probability < 0.3)
- **1:** Unknown (probability 0.3-0.7)
- **2:** Occupied (probability > 0.7)

### World ↔ Grid Transformations

```python
# occupancy_grid.py

def world_to_grid(x_m, y_m):
    """Convert world coordinates to grid indices"""
    center_x = width // 2
    center_y = height // 2
    gx = int(center_x + (x_m - origin_offset_x) / resolution)
    gy = int(center_y - (y_m - origin_offset_y) / resolution)  # Y inverted
    return gx, gy

def grid_to_world(gx, gy):
    """Convert grid indices to world coordinates"""
    center_x = width // 2
    center_y = height // 2
    x_m = (gx - center_x) * resolution + origin_offset_x
    y_m = (center_y - gy) * resolution + origin_offset_y  # Y inverted
    return x_m, y_m
```

---

## Arduino Frame (Encoder/Motor)

The **Arduino frame** represents encoder and motor data from the motor controller.

### Encoder Convention

- **Left wheel positive:** Robot moves forward
- **Right wheel positive:** Robot moves forward
- **Left > Right:** Robot turns right (CW from above)
- **Right > Left:** Robot turns left (CCW from above)

### Parameters

```python
# Calibrated values (PROJECT_STATUS.md)
wheel_diameter = 79 mm
wheelbase = 244 mm
ticks_per_revolution = 714 ticks/rev
distance_per_tick = π * 0.079 / 714 = 0.000348 m/tick
```

### Odometry Integration

```python
# motion_model.py
d_left = delta_left * distance_per_tick
d_right = delta_right * distance_per_tick
d_center = (d_left + d_right) / 2.0
d_theta = (d_right - d_left) / wheelbase

# Forward kinematics (world frame)
dx = d_center * cos(theta)
dy = d_center * sin(theta)
new_theta = theta + d_theta
```

---

## LIDAR Frame

The **LIDAR frame** describes LIDAR scan data before transformation.

### Angle Convention

- **0°:** Robot forward (+X in robot frame)
- **Positive rotation:** Counter-clockwise (CCW)
- **Range:** Typically 0° to 359°
- **Example:**
  - 0° = Forward
  - 90° = Left
  - 180° = Backward
  - 270° = Right

### Transform to World Frame

```python
# occupancy_grid.py (line 214-219)
beam_angle_rad = math.radians(reading.angle_deg) + robot_pose.theta
hit_x = robot_pose.x + dist_m * math.cos(beam_angle_rad)
hit_y = robot_pose.y + dist_m * math.sin(beam_angle_rad)
```

### Data Structure

```python
@dataclass
class LidarReading:
    angle_deg: float     # Beam angle in degrees (0-359)
    distance_mm: int     # Range in millimeters
    quality: int         # Measurement quality (0-255)
    valid: bool          # True if reading is reliable
```

---

## Transformations

### 1. Robot Frame → World Frame

```python
def transform_robot_to_world(points_robot, robot_pose):
    """
    Args:
        points_robot: Nx2 array [[x1,y1], [x2,y2], ...] in robot frame
        robot_pose: Pose(x, y, theta) in world frame

    Returns:
        Nx2 array in world frame
    """
    # Rotation matrix
    c = np.cos(robot_pose.theta)
    s = np.sin(robot_pose.theta)
    R = np.array([[c, -s],
                  [s,  c]])

    # Transform: rotate then translate
    points_world = points_robot @ R.T
    points_world[:, 0] += robot_pose.x
    points_world[:, 1] += robot_pose.y

    return points_world
```

### 2. World Frame → Grid Frame

See [Grid Frame section](#world-grid-transformations) above.

### 3. Grid Frame → Canvas Pixels

```javascript
// occupancy_map.js
const cellWidth = canvas.width / grid_width;
const cellHeight = canvas.height / grid_height;

const pixelX = gx * cellWidth;
const pixelY = gy * cellHeight;
```

### 4. Canvas Pixels → World Frame

```javascript
// occupancy_map.js (pixelToWorld)
const gx = pixelX / cellWidth;
const gy = pixelY / cellHeight;

const worldX = (gx - width/2) * resolution + originOffsetX;
const worldY = (height/2 - gy) * resolution + originOffsetY;
```

---

## Visualization Conventions

### HTML5 Canvas Coordinate System

```
(0, 0) ●----------------→ +X (canvas)
       |
       |
       |
       ↓
      +Y (canvas)
```

- **Origin:** Top-left corner
- **X-axis:** Right (matches world +X)
- **Y-axis:** Down (inverted from world +Y)

### Y-Axis Inversion Handling

The canvas Y-axis points down, while world Y points up. This is handled in two places:

1. **Grid conversion:**
   ```python
   gy = center_y - (y_m - origin_offset_y) / resolution  # Subtract instead of add
   ```

2. **Robot rendering:**
   ```javascript
   ctx.rotate(-pose.theta);  // Negative because Y is flipped
   ```

### Visual Reference

**On screen (canvas):**
- **Upward movement** = Positive world Y = Heading 90°
- **Rightward movement** = Positive world X = Heading 0°
- **Robot triangle points** in direction of heading

**Color conventions:**
- **White:** Free space (robot can traverse)
- **Gray:** Unknown (not yet explored)
- **Black:** Occupied (obstacle detected)
- **Green:** Robot position and heading
- **Red:** Origin marker (world 0,0)
- **Light gray lines:** 1m grid spacing

---

## Common Pitfalls and Debugging Tips

### 1. Y-Axis Confusion

**Problem:** Robot appears to move in wrong direction on canvas.

**Check:**
- Is Y inverted in grid_to_world? (Should subtract, not add)
- Is robot heading negated in canvas rotation?
- Are you mixing world Y and grid Y?

### 2. Origin Offset Errors

**Problem:** Robot appears at wrong location on map.

**Check:**
- Is origin_offset subtracted in world_to_grid?
- Is origin_offset added in grid_to_world?
- Does frontend receive and use originOffsetX/Y?

### 3. Heading Convention Mismatch

**Problem:** LIDAR obstacles appear rotated.

**Check:**
- LIDAR angle is in degrees, robot theta in radians
- Are you adding robot heading to beam angle?
- Is cos/sin applied to correct axes?

### 4. Encoder Direction

**Problem:** Robot pose drifts or moves opposite of commanded.

**Check:**
- Encoder signs: positive should be forward motion
- Wheelbase value: affects turning radius
- Distance per tick: affects speed/distance scaling

---

## Testing and Validation

### Visual Checks

1. **Straight line test (theta=0°):**
   - Robot should move RIGHT on screen
   - X should increase, Y unchanged

2. **Straight line test (theta=90°):**
   - Robot should move UP on screen
   - Y should increase, X unchanged

3. **Rotation test:**
   - Right wheel forward → Turn left (CCW, theta increases)
   - Left wheel forward → Turn right (CW, theta decreases)

4. **Origin marker:**
   - Should stay at red cross on canvas
   - Should correspond to world (0, 0)

### Numerical Checks

```python
# Test world ↔ grid round-trip
x, y = 1.0, 2.0
gx, gy = world_to_grid(x, y)
x2, y2 = grid_to_world(gx, gy)
assert abs(x - x2) < 0.03  # Within one cell
assert abs(y - y2) < 0.03
```

---

## References

- **motion_model.py:** Differential drive kinematics and odometry integration
- **occupancy_grid.py:** World ↔ Grid transformations and LIDAR mapping
- **occupancy_map.js:** Canvas rendering and pixel ↔ World transformations
- **sensor_data.py:** Pose and LidarReading data structures
- **PROJECT_STATUS.md:** Calibrated robot parameters

---

## Revision History

- **2026-01-09:** Initial version with configurable origin offset support
- **Future:** Add IMU frame, multi-robot coordination frames
