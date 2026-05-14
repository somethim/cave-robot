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
                     ↓          ↓
           crates/slam   crates/pathfinding
                     ↓          ↓
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
| [docs/gazebo.md](docs/gazebo.md)             | Gazebo world pipeline plan           |
| [docs/setup.md](docs/setup.md)               | Build, install, and usage            |
| [docs/architecture.md](docs/architecture.md) | Full project structure and crate map |

## Resources

- [Cellular Automata Method for Generating Random Cave-Like Levels](https://www.roguebasin.com/index.php/Cellular_Automata_Method_for_Generating_Random_Cave-Like_Levels)
  by RogueBasin — the 2D CA algorithm used per slice
- [Marching Cubes: A High Resolution 3D Surface Construction Algorithm](https://dl.acm.org/doi/10.1145/37402.37422)
  by Lorensen & Cline — voxel-to-mesh conversion for Gazebo worlds
- [Amit's A* Pages](https://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html)
  by Patel — practical introduction to A* and related algorithms
- [A* and D* Lecture](https://www.cs.cmu.edu/~motionplanning/lecture/AppH-astar-dstar_howie.pdf)
  (CMU) — covers both A* and D* in one appendix
- [`rclrs`](https://docs.rs/rclrs) — ROS 2 Rust client library
