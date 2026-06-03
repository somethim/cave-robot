# Gazebo Integration

## Overview

The Gazebo simulation is fully working end-to-end. `just all` or `just bringup`
launches the complete stack: Gazebo sim, ROS bridge, and robot node.

## World generation pipeline

```
cave.json ──→ cave_to_world ──→ cave.world (SDF)
                                     │
                               Gazebo Harmonic
                                     │
                       GPU LiDAR + VelocityControl plugin
                                     │
                          ros_gz_bridge (LaserScan, Twist)
                                     │
                           cave_robot_node (rclrs)
                                     │
                          robot binary subprocess (--pipe)
```

`cave_to_world` (`apps/generator/src/bin/cave_to_world.rs`) reads `cave.json`
and writes:
- `cave.world` — Gazebo SDF world
- `cave.meta.json` — sidecar with start/end cells and robot spawn position

## Cave world generation

### Wall geometry

Wall cells are merged into maximal rectangles using a greedy row-then-column
packing algorithm. Each merged rectangle becomes one SDF `<box>` collision and
visual, spanning the full level height (floor to ceiling). This reduces the
number of physics objects in Gazebo compared to one box per cell.

### Floor and ceiling slabs

All passable floor cells (non-ramp, non-hole) get a thin `0.1 m` floor slab at
the bottom of the level and a ceiling slab at the top. These give the LiDAR
surfaces to hit vertically and define the navigable volume.

### Ramps and holes

`TILE_RAMP` cells: a rotated box at 45° pitch, sized to span the full 2 m level
gap. Provides a visual indicator of the vertical connection.

`TILE_HOLE` cells: four thin wall panels forming a square shaft. The drone flies
through the opening; the panels define the shaft walls.

### Markers

Start and end positions have coloured sphere markers (radius 0.18 m). They are
visual-only — no `<collision>` element — so the drone passes through them without
physical contact.

### Lighting

One point light per 8 × 8 cell grid sector at each level, placed at the first
passable cell found in each sector near the ceiling. Provides in-cave visibility
without blowing out Gazebo's renderer.

## Drone model

**File:** `ros/src/cave_robot_description/urdf/cave_robot.sdf`

| Component | Detail |
|---|---|
| Frame | Two crossed 0.30 m bars at ±45° (X-frame visual) |
| Rotors | 4 cylinders at ±0.09 m from centre, radius 0.036 m |
| Collision | Single box 0.28 m × 0.28 m × 0.08 m |
| LiDAR mount | Fixed joint 0.15 m above base; separate `lidar_link` |
| Gravity | Disabled — drone hovers without thrust model |

**LiDAR sensor** (`lidar_link`):

| Parameter | Value |
|---|---|
| Horizontal samples | 180 (360°, −π to π) |
| Vertical samples | 32 (±40°, −0.698 to 0.698 rad) |
| Range | 0.1 m – 8.0 m |
| Update rate | 10 Hz |
| Topic | `/cave_robot/lidar` |

The ±40° vertical arc (80° total) produces a cylindrical scan volume — enough
to detect floors, ceilings, and walls simultaneously.

**Gazebo plugins:**

| Plugin | Role |
|---|---|
| `gz-sim-velocity-control-system` | Kinematic motion from `cmd_vel` Twist |
| `gz-sim-pose-publisher-system` | Streams model pose to `/model/cave_robot/pose` |
| `gz-sim-sensors-system` | Drives GPU LiDAR rendering (ogre2) |

The `VelocityControl` plugin moves the model kinematically matching the robot
binary's motion model exactly: `linear.x`, `linear.z`, `angular.z` map directly
to the drone's body-frame velocities.

## ROS bridge topics

| Topic | Type | Direction | Role |
|---|---|---|---|
| `/cave_robot/lidar` | `sensor_msgs/LaserScan` | Gazebo → ROS | LiDAR ranges |
| `/model/cave_robot/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo | Velocity command |
| `/model/cave_robot/pose` | (gz internal) | Gazebo → ROS | Pose publisher |

Note: the `ros_gz_bridge` converts the GPU LiDAR output to a 2-D `LaserScan`
(horizontal only). The vertical dimension is lost — `angle_min_vertical`,
`angle_max_vertical`, `angle_increment_vertical` are all set to 0 in the node.
The robot's 3-D LiDAR simulation is only available in offline mode.

## Scale and coordinate conventions

| Constant | Value |
|---|---|
| Voxel size (x/y) | 1.0 m |
| Level spacing (z) | 2.0 m |
| Cell centre z bias | 0.5 m |
| Robot spawn z offset | 0.25 m |

Mapping from grid cell `(x, y, z)` to Gazebo world coordinates:

```
wx = (x + 0.5) * 1.0
wy = (y + 0.5) * 1.0
wz = z * 2.0 + 0.5 + 0.25    (spawn)
wz = z * 2.0 + 0.5           (cell centre)
```

## Launching

```bash
just all           # full build + generate + bringup (first time)
just bringup       # launch Gazebo + bridge + robot node (already built)
just sim           # Gazebo GUI only, no robot node
```

Individual steps:

```bash
just generate        # cave.json from generator
just generate-world  # cave.world + cave.meta.json from cave.json
just ros-build       # incremental colcon build
just bringup         # ros2 launch cave_robot_bringup bringup.launch.py
```

### World file resolution

The launch file resolves the world path in this order:

1. `CAVE_WORLD_FILE` environment variable
2. `ros/src/cave_robot_gazebo/worlds/generated/cave.world` (source tree)
3. Installed copy under `share/cave_robot_gazebo/worlds/generated/`

### Notes

- `just bringup` uses the `ros-run` script to source `/opt/ros/jazzy/setup.bash`
  before launching, so ROS discovery works inside the distrobox.
- Gazebo forces X11/XCB rendering (`QT_QPA_PLATFORM=xcb`) for distrobox
  compatibility.
- The physics engine is DART with `max_step_size=0.01 s` and `real_time_factor=1.0`.
