# Architecture

## Project Structure

```
├── apps/
│   ├── generator/        # Cave map generator (Rust)
│   └── robot/            # Robot control loop (Rust, offline / JSON mode)
├── crates/
│   ├── shared/           # Shared types: Pose, CaveMap, RobotState, etc.
│   ├── pathfinding/      # D* Lite (forward), A* (return)
│   └── slam/             # SLAM state estimation (particle filter / EKF)
├── ros/
│   ├── scripts/
│   │   └── ros-run                # Helper to source ROS setup then run a command
│   ├── setup.sh                   # One-shot build script (submodules + colcon)
│   ├── deps/                      # Git submodules
│   │   ├── common_interfaces/     #   sensor_msgs, geometry_msgs, std_msgs
│   │   ├── example_interfaces/
│   │   ├── rcl_interfaces/
│   │   ├── rosidl_core/
│   │   ├── rosidl_defaults/
│   │   ├── unique_identifier_msgs/
│   │   └── rosidl_rust/           #   Rust ROS IDL code generator
│   └── src/
│       ├── cave_robot_bringup/    # ROS 2 launch files
│       ├── cave_robot_description/ # URDF / robot model
│       ├── cave_robot_gazebo/     # Gazebo simulation worlds
│       ├── cave_robot_msgs/       # Custom ROS 2 messages
│       └── cave_robot_node/       # ROS-integrated Rust node (rclrs)
├── Cargo.toml            # Rust workspace root (non-ROS code)
└── justfile              # Convenience commands
```

## Data Flow

### Offline mode

```
generator → cave.json → robot (reads from disk)
```

### ROS 2 mode

```
generator → cave.json → generated SDF world + embedded robot → Gazebo
                                                          ↓
                                         ros_gz bridge + cave_robot_node (target)
```

The cave generator runs once to produce `cave.json`. On the ROS path,
`cave.json` is converted to a Gazebo SDF world — the robot **never reads
`cave.json` directly**. It discovers the environment purely through `/scan`
data, building a SLAM map from scratch.

Current implementation status:

- `cave.json` is converted into a generated Gazebo world under
  `ros/src/cave_robot_gazebo/worlds/generated/`
- The generated world embeds the robot model at `cave.start`
- Gazebo visual inspection works through `just sim`
- The ROS topic bridge and full simulation bringup are not finished yet

## ROS Bridge

The ROS ↔ Rust bridge uses `rclrs` (v0.7), the official ROS 2 Rust client library. Message Rust bindings are
auto-generated at build time from the original `.msg` files using `rosidl_rust`.

### Key Topics

| Topic      | Type                    | Direction     |
|------------|-------------------------|---------------|
| `/scan`    | `sensor_msgs/LaserScan` | Gazebo → Rust |
| `/cmd_vel` | `geometry_msgs/Twist`   | Rust → Gazebo |

### Build System

The `cave_robot_node` is an `ament_cargo` package built by `colcon`. Colcon:

1. Generates Rust bindings for all message packages (`sensor_msgs`, etc.)
2. Writes a `.cargo/config.toml` that patches the yanked crates.io message crates with the locally generated versions
3. Builds `cave_robot_node` with those local bindings

## Navigation

### Forward (Start → End): D* Lite

The drone assumes all unknown cells are traversable. As LIDAR sweeps reveal walls and open space, D\* Lite efficiently
repairs only the affected portion of the path rather than replanning from scratch. This gives real-time performance in
partially known environments.

### Return (End → Start): A\*

After the forward journey, the drone has a fully explored map built by SLAM. With complete knowledge, A\* computes the
optimal shortest path back to start. No LIDAR is available.

### 3D Pathfinding

Both D\* Lite and A\* operate on a 26-connected 3D voxel grid (x, y, z). The
drone moves freely in 3D space — ramps connect z-slices, replacing stair
teleports. See [generation.md](generation.md) for the cave design.
