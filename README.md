# Cave Robot

An autonomous drone that navigates randomly generated cave systems using LiDAR, SLAM, and pathfinding.

The drone is placed inside a procedurally generated cave with the full map known in advance. It navigates
from **start → end** using **A\*** on a clearance-weighted graph, with an **Extended Kalman Filter** fusing
motor dead-reckoning with Gazebo pose measurements for localisation and **SLAM** building an occupancy map
from LiDAR scans along the way. Once it reaches the destination, it navigates back **end → start** using
**A\*** planned on the SLAM-derived map — the path the robot earned by observation rather than the ground truth.

The bridge between Rust navigation logic and ROS 2 / Gazebo uses [`rclrs`](https://docs.rs/rclrs).

## Architecture

```
apps/generator  ──→  cave.json  ──→  apps/robot  (offline / pipe mode)
                          │
                    cave_to_world
                          │
                          ▼
             ros/src/cave_robot_gazebo/worlds/
                          │
         Gazebo ←── ros_gz_bridge ←── cave_robot_node
           │                               │
       GPU LiDAR                     spawns robot binary
       pose publisher                as subprocess (--pipe)
           │                               │
           └───── stdin/stdout JSON ───────┘
```

### Crates

| Crate | Role |
|---|---|
| `apps/generator` | Procedural cave generator — cellular automata, connectivity, JSON export |
| `apps/robot` | Navigation binary — A\*, EKF, SLAM loop, pipe protocol |
| `crates/shared` | Common types: `Cave`, `Pose`, `LidarScan`, `OccupancyGrid`, coordinate helpers |
| `crates/pathfinding` | A\* and D\* Lite on a 3-D cardinal-direction grid with per-cell costs |
| `crates/slam` | Particle-filter SLAM — occupancy grid map built from LiDAR during forward trip |
| `crates/kalman` | Extended Kalman Filter — fuses dead-reckoning with Gazebo pose measurements |
| `ros/src/cave_robot_node` | ROS 2 bridge — LiDAR → robot stdin, robot stdout → cmd\_vel |

## Navigation summary

| Phase | Direction | Algorithm | Localisation |
|---|---|---|---|
| Forward | Start → End | A\* on clearance-weighted known graph | EKF (dead-reckoning + Gazebo pose fusion) |
| Return | End → Start | A\* on SLAM-derived occupancy map | EKF / Gazebo pose |

## Docs

| File | What |
|---|---|
| [docs/generation.md](docs/generation.md) | Cave generation algorithm |
| [docs/navigation.md](docs/navigation.md) | Forward & return navigation, EKF, SLAM |
| [docs/gazebo.md](docs/gazebo.md) | Gazebo world pipeline and ROS bridge |
| [docs/architecture.md](docs/architecture.md) | Full project structure and data flow |
| [docs/setup.md](docs/setup.md) | Build, install, and usage |

## Quick start

```bash
distrobox enter cave-robot-ros-two
just all        # build → generate cave → generate world → bringup
```

Or step by step:

```bash
just build           # cargo + colcon
just generate        # cave.json
just generate-world  # Gazebo SDF world
just bringup         # Gazebo + ROS bridge + robot node
```

Offline (no Gazebo):

```bash
just generate
just robot           # reads CAVE_FILE, simulates LiDAR internally
```

## Resources

- [Cellular Automata Method for Generating Random Cave-Like Levels](https://www.roguebasin.com/index.php/Cellular_Automata_Method_for_Generating_Random_Cave-Like_Levels)
  — the 2-D CA algorithm used per z-slice
- [Amit's A\* Pages](https://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html)
  — practical introduction to A\* and related algorithms
- [A\* and D\* Lecture (CMU)](https://www.cs.cmu.edu/~motionplanning/lecture/AppH-astar-dstar_howie.pdf)
  — covers both A\* and D\* in one appendix
- [Probabilistic Robotics — Thrun, Burgard, Fox](http://www.probabilistic-robotics.org/)
  — particle filter SLAM and EKF derivations
- [`rclrs`](https://docs.rs/rclrs) — ROS 2 Rust client library
