# Cave Robot

An autonomous drone that navigates randomly generated cave systems using LIDAR, SLAM, and pathfinding.

The drone is placed inside a procedurally generated cave with only the start and end known — no prior map. Using a LIDAR
system, it navigates from start to end via **D\* Lite** (real-time replanning as obstacles are revealed). Once it
reaches the destination, the LIDAR is cut off and the drone must navigate back to the start using only its stored map
via a **SLAM** algorithm, computing the optimal path with **A\***.

The bridge between Rust navigation logic and ROS 2 / Gazebo uses [`rclrs`](https://docs.rs/rclrs).

## Architecture

```
apps/generator  →  crates/shared  →  apps/robot  (offline / JSON mode)
                     ↓
crates/pathfinding ← crates/slam
                     ↓
ros/src/cave_robot_node  ←→  ROS 2 topics  ←→  Gazebo
        (rclrs)
```

- **Cave generation** — 2D cellular automata per z-slice with 3D ramp connectivity
- **Pathfinding** — D\* Lite for forward navigation, A\* for return
- **SLAM** — state estimation from LIDAR data for return navigation
- **ROS bridge** — `rclrs` node subscribes to LIDAR scans, publishes velocity commands

## Docs

| File                                         | What                                 |
|----------------------------------------------|--------------------------------------|
| [docs/generation.md](docs/generation.md)     | Cave generation algorithm            |
| [docs/setup.md](docs/setup.md)               | Build, install, and usage            |
| [docs/architecture.md](docs/architecture.md) | Full project structure and crate map |

## Resources

- [Cellular Automata Method for Generating Random Cave-Like Levels](https://www.roguebasin.com/index.php/Cellular_Automata_Method_for_Generating_Random_Cave-Like_Levels)
  by RogueBasin
