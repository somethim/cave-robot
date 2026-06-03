# Architecture

## Project structure

```
cave-robot/
├── apps/
│   ├── generator/            # Procedural cave generator
│   │   ├── src/
│   │   │   ├── main.rs       # Entry point — reads env, calls Map::generate, writes JSON
│   │   │   ├── map.rs        # Map struct, GeneratorConfig, generate() pipeline
│   │   │   ├── map/
│   │   │   │   ├── init.rs       # Random fill + border walls
│   │   │   │   ├── cellular.rs   # CA smoothing step
│   │   │   │   ├── connect.rs    # Region flood-fill + union-find connectivity
│   │   │   │   ├── dead_ends.rs  # Dead-end tunnel carving
│   │   │   │   ├── ramps.rs      # Inter-level ramp placement
│   │   │   │   ├── holes.rs      # Vertical shaft placement
│   │   │   │   ├── flood_fill.rs # 2-D and 3-D flood fill labelling
│   │   │   │   └── start_end.rs  # Random start/end selection
│   │   │   └── bin/
│   │   │       └── cave_to_world.rs  # Converts cave.json → Gazebo SDF world
│   │   └── src/world.rs      # SDF generation: wall/slab/ramp/hole/marker geometry
│   └── robot/
│       └── src/main.rs       # Full navigation loop (A*, EKF, SLAM, pipe protocol)
├── crates/
│   ├── shared/
│   │   └── src/
│   │       ├── cave.rs       # Cave struct, tile constants, coordinate helpers
│   │       ├── robot.rs      # Pose, LidarScan, OccupancyGrid, NavCommand, RobotState
│   │       └── lib.rs        # env_or, load_env, re-exports
│   ├── pathfinding/
│   │   └── src/lib.rs        # A* + D* Lite on GridGraph (3-D cardinal neighbours)
│   ├── slam/
│   │   └── src/lib.rs        # Particle-filter SLAM: predict/update/resample + occupancy grid
│   └── kalman/
│       └── src/lib.rs        # Extended Kalman Filter: predict + measurement update
└── ros/
    ├── scripts/
    │   ├── ros-bootstrap     # Init submodules, build external message packages
    │   ├── ros-build         # Incremental colcon build of project packages
    │   └── ros-run           # Source ROS setup then exec a command
    ├── deps/                 # Git submodules: ROS message definitions + rosidl_rust
    └── src/
        ├── cave_robot_bringup/     # Launch files
        ├── cave_robot_description/ # Drone URDF + SDF model
        ├── cave_robot_gazebo/      # Gazebo launch + generated worlds
        └── cave_robot_node/        # rclrs node — LiDAR bridge, robot subprocess
```

## Data flow

### Offline (no ROS)

```
generator ──→ cave.json ──→ robot binary (--simulates LiDAR internally--)
                                │
                        stdout: step logs
```

### ROS / Gazebo

```
generator ──→ cave.json ──→ cave_to_world ──→ cave.world (SDF)
                                                    │
                                              Gazebo sim
                                                    │
                               ┌────────────────────┤
                               │                    │
                         GPU LiDAR            pose publisher
                         /cave_robot/lidar    /model/.../pose
                               │                    │
                          ros_gz_bridge        ros_gz_bridge
                               │                    │
                      ROS LaserScan           ROS Pose topic
                               │
                     cave_robot_node
                       (rclrs node)
                               │
                    ┌──────────┴──────────┐
                    │                     │
               stdin (JSON scan)    stdout (JSON cmd)
                    │                     │
               robot binary          geometry_msgs/Twist
               (--pipe mode)              │
                    │                ros_gz_bridge
                    └──────────────────── │
                                    Gazebo cmd_vel
```

## Key types (`crates/shared`)

| Type | Where | Role |
|---|---|---|
| `Cave` | `cave.rs` | 3-D grid `[z][y][x]` of `u8` tiles + start/end cells |
| `Pose` | `robot.rs` | `{x, y, z, yaw}` in cave-grid coordinates |
| `LidarScan` | `robot.rs` | Range array + angle metadata + optional embedded `Pose` |
| `OccupancyGrid` | `robot.rs` | Log-odds 3-D grid with Bresenham ray update |
| `NavCommand` | `robot.rs` | `{linear_x, linear_y, linear_z, angular_z}` velocity command |

## Physical scale

Defined in `crates/shared/src/cave.rs`:

| Constant | Value | Meaning |
|---|---|---|
| `GAZEBO_VOXEL_SIZE_METERS` | 1.0 m | Width/depth of one grid cell |
| `GAZEBO_LEVEL_SPACING_METERS` | 2.0 m | Height between z-levels |
| `GAZEBO_CELL_CENTER_Z_BIAS` | 0.5 m | Vertical offset from level base to cell centre |
| `GAZEBO_ROBOT_SPAWN_Z_OFFSET` | 0.25 m | Extra clearance above cell centre at spawn |

Cell centre in Gazebo for grid cell `(x, y, z)`:

```
world_x = (x + 0.5) * 1.0
world_y = (y + 0.5) * 1.0
world_z = z * 2.0 + 0.5 + 0.25   (spawn)
```

## ROS bridge

The `cave_robot_node` spawns the robot binary as a child process with `--pipe`.
All ROS dependency is isolated to the node; the robot binary has none.

**LiDAR path (Gazebo → robot):**

`Gazebo gpu_lidar` → `ros_gz_bridge` → ROS `sensor_msgs/LaserScan`
→ `cave_robot_node` deserialises → `LidarScan` JSON → robot `stdin`

The LaserScan from the bridge is 2-D (horizontal only) because `sensor_msgs/LaserScan`
has no vertical field. Vertical angles are set to zero; the robot receives a
single-row scan. The simulated LiDAR in offline mode is full 3-D (32 vertical
rays, ±40°) and is used for SLAM map building.

**Command path (robot → Gazebo):**

Robot `stdout` JSON `{linear_x, linear_z, angular_z}`
→ `cave_robot_node` → `geometry_msgs/Twist`
→ `ros_gz_bridge` → Gazebo `VelocityControl` plugin

**Pipe lifecycle:**
An `AtomicBool` (`robot_done`) is set when the robot's stdout closes (mission
complete). Subsequent LiDAR callbacks check it and skip writing to the now-closed
stdin, eliminating broken-pipe errors.

## Pathfinding (`crates/pathfinding`)

Two algorithms share a `GridGraph` type:

**A\*** — used for both forward and return planning. Runs once on a complete cost
map and finds the optimal path. Standard open-set / came-from implementation with
an octile-distance 3-D heuristic.

**D\* Lite** — implemented but not used for forward navigation. D\* Lite's
advantage is incremental replanning: when a small number of edges change it
repairs only the affected region, O(k) rather than O(n log n) full replan.
This matters when the map is *discovered* incrementally — the robot assumes unknown
cells are free and updates the planner each time a new wall is seen.

In this system the cave is fully known before navigation starts (loaded from
`cave.json` at startup). There are no unknown cells and no incremental discoveries
to trigger D\* Lite's repair step. A\* on the complete cost map is therefore both
simpler and sufficient — D\* Lite's incremental machinery would never fire.

**Clearance weighting:** Before A\* runs, a BFS flood-fill from all wall cells
computes distance-to-nearest-wall for each passable cell. Cell costs are set to
6 × (adjacent to wall), 2.5 × (one step away), or 1 × (clear). This routes
paths through corridor centres without needing wider passages.

**Cardinal-only neighbours:** Diagonal moves are excluded to prevent the drone's
body clipping the inner corner of a wall during a turn.
