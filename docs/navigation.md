# Robot Navigation System

## Overview

The robot navigates a procedurally generated 3D cave in two phases:

1. **Forward phase** (Start → End): D\* Lite incremental replanning with simulated LIDAR, treating unknown space as traversable. Discovers walls in real time and replans around them.

2. **Return phase** (End → Start): A\* on the SLAM-built occupancy grid. The map is complete, so a single optimal path suffices. No LIDAR available during return.

This two-phase strategy exists because forward navigation happens in an unknown environment where D\* Lite's incremental replanning is ideal, while the return uses the accumulated map for a direct path.

```
                    ┌──────────────────┐
                    │  Cave Generator  │
                    │  (apps/generator)│
                    └──────┬───────────┘
                           │ cave.json
                           ▼
          ┌────────────────┴────────────────┐
          │                                 │
          ▼                                 ▼
   Offline mode                        ROS/Gazebo mode
   ┌──────────────┐              ┌──────────────────────┐
   │ robot binary  │              │ cave → SDF world     │
   │ reads cave    │              │ Gazebo simulates     │
   │ simulates     │              │ GPU LiDAR + physics  │
   │ LIDAR + moves │              │ ros_gz_bridge relays │
   └──────┬───────-┘              │ topics               │
          │                      │ rclrs node runs      │
          │                      │ robot as subprocess  │
          │                      └──────────┬───────────┘
          │                                 │
          ▼                                 ▼
   ┌──────────────────────────────────────────────┐
   │              Navigation Loop                  │
   │  ┌─────────────┐    ┌──────────────────┐     │
   │  │ D* Lite     │    │  SLAM particle   │     │
   │  │ pathfinding │◄──►│  filter + map    │     │
   │  └──────┬──────┘    └────────┬─────────┘     │
   │         │                    │               │
   │         ▼                    ▼               │
   │  ┌──────────────────────────────────┐        │
   │  │  OccupancyGrid (log-odds, 3D)    │        │
   │  │  Bresenham 3D ray casting        │        │
   │  └──────────────────────────────────┘        │
   └──────────────────────────────────────────────┘
```

## Forward Phase: D\* Lite

### What

D\* Lite is an incremental heuristic search algorithm. Given a graph with a known start and goal, it computes an optimal path. When obstacles are discovered (the graph changes), D\* Lite reuses previous search results to repair only the affected region of the path, rather than replanning from scratch.

### How

Implementation in `crates/pathfinding/src/lib.rs`:

```
compute_shortest_path():
  loop:
    top_key = min key in priority queue
    if top_key >= start_key AND start is consistent:
      return (path is optimal)
    pop node u with minimum key from queue
    if g(u) > rhs(u):
      g(u) = rhs(u)    # overconsistent → set to rhs
      update all predecessors of u
    else:
      g(u) = ∞         # underconsistent → expand
      update all predecessors of u
      update u itself
```

Key differences from textbook D\* Lite:
- **26-connected 3D graph**: neighbors include all 26 adjacent voxels (axial, diagonal, and triagonal). Edge cost = `distance × cost(voxel)`, where passable voxels cost 1.0 and blocked voxels cost ∞.
- **Octile-3D heuristic**: `d3·√3 + (d2−d3)·√2 + (d1−d2)·1.0` where d1 ≥ d2 ≥ d3 are the axis-aligned distances. This is admissible (never overestimates) and consistent, guaranteeing optimal paths.
- **Movement model**: The robot tracks a continuous pose with `run_forward()`. At each step it simulates LIDAR, discovers blocked cells, calls `update_obstacle()` on each newly discovered wall, then calls `move_start()` to update the planner's start to the current cell. `get_path()` reuses the search from previous iterations.

### Why D\* Lite and not A\* for forward?

A\* replans from scratch every time the map changes — O(n log n) per replan. D\* Lite only processes the affected region, which is typically a small fraction of the graph when a nearby wall is discovered. This matters because the robot performs one replan per LIDAR scan (each scan may discover multiple new obstacles).

### Why treat unknown as traversable?

The robot has no prior map of the environment. It assumes all cells are passable until LIDAR proves otherwise. If the assumption is wrong, D\* Lite replans. This is standard for unknown-environment navigation — treat the unknown as free, detect walls as you go.

## Return Phase: A\*

### What

A\* is a complete, optimal best-first search using a heuristic. Since the SLAM map is fixed during the return journey (no LIDAR available), there is no need for incremental replanning — A\* finds the optimal return path in one pass.

### How

Standard A\* implementation in `crates/pathfinding/src/lib.rs`:

```
astar(graph, start, goal):
  open set = BinaryHeap ordered by f = g + h
  g(start) = 0
  pop node with minimum f
  for each passable neighbor:
    if g(current) + edge_cost < g(neighbor):
      update g(neighbor), set parent, push to open
  continue until goal is popped or open is empty
```

The A\* uses the same octile-3D heuristic as D\* Lite.

### Why A\* instead of D\* Lite for return?

The return phase has no sensor input, so the map never changes. A\* finds a single optimal path in one pass — simpler code, no overhead of maintaining g/rhs/queue across updates. D\* Lite's incremental machinery is unnecessary when the graph is static.

## LIDAR Simulation

### What

A simulated 3D LIDAR scanner that casts rays through the cave voxel grid and returns measured ranges.

### How

`LidarScan::simulate()` in `crates/shared/src/robot.rs`:

- Default config: 180 horizontal rays × 16 vertical rays, 180° horizontal FOV, 34° vertical FOV, 8m max range.
- For each (azimuth, elevation) pair, `cast_ray_3d()` steps 0.1m along the ray until it hits a wall or exceeds max range.
- Gaussian noise (stddev 0.05m) is added to each range.
- The scan is serializable to JSON for the ROS pipe protocol.

```rust
fn cast_ray_3d(origin, azimuth, elevation, max_range, cave) -> range:
    step = 0.1
    loop:
        dist += step
        if dist > max_range: return max_range
        cell = voxel_at(origin + direction * dist)
        if out of bounds: return dist
        if cell is wall: return dist
```

### Why a simple step-based ray cast?

A 3D DDA (Digital Differential Analyzer) could step voxel-to-voxel faster, but 180×16 = 2880 rays × 0.1m steps × 8m range = ~230k checks per scan is well within budget for the offline and pipe modes. The step size of 0.1m (1/10 of a voxel) gives sufficient accuracy for navigation.

## Occupancy Grid

### What

A 3D probabilistic occupancy grid using log-odds representation. Each cell stores log-odds = `log(p / (1-p))`, where p is the probability the cell is occupied. Unknown cells have log-odds 0, meaning p = 0.5.

### How

`OccupancyGrid` in `crates/shared/src/robot.rs`:

- **Update**: `update_cell(x, y, z, occupied)` adds `LOG_ODDS_OCCUPIED` (0.8) or `LOG_ODDS_FREE` (−0.4) to the cell's log-odds.
- **Ray update**: `update_ray(x0,y0,z0, x1,y1,z1)` iterates over all cells along the Bresenham 3D ray from origin to endpoint. All intermediate cells (including the origin) are marked free. The endpoint is marked occupied. Cells with probability > 0.5 threshold are considered occupied.
- **Grid conversion**: `to_passable_grid()` returns a 3D bool array: false for occupied, true for free/unknown.

Bresenham 3D algorithm (`bresenham_3d`):

```
based on the axis with the largest delta:
  step along the major axis
  accumulate errors for minor axes
  when error ≥ 0, step along that minor axis
```

This produces a conservative, hole-free line of voxels between two grid cells — suitable for ray casting in an occupancy grid.

### Why log-odds?

Log-odds simplifies Bayesian fusion: sensor updates are additive. Multiple observations of the same cell accumulate evidence without needing to track counts. A cell seen free 10 times and occupied 1 time has log-odds = 10×(−0.4) + 1×0.8 = −3.2, probability ≈ 0.04 — correctly free.

### Why skip marking origin as occupied?

The origin cell is the robot's position. A ray that terminates at the robot's own cell (laser hits something at the origin) should not mark it occupied, since the robot must be on free space. The `update_ray` method explicitly checks `last != origin` before marking the endpoint as occupied.

## SLAM

### What

A particle filter SLAM system (200 particles) that simultaneously estimates the robot's pose and builds an occupancy grid map from LIDAR scans.

### How

`Slam` in `crates/slam/src/lib.rs`:

**Initialization**: 200 particles are spawned around the initial pose with Gaussian noise (±0.25m xy, ±0.15m z, ±0.1 rad yaw).

**Update cycle** (called per LIDAR scan):

1. **Motion model** (`predict`): Each particle's pose is propagated forward using the control command (linear+angular velocity) plus noise proportional to motion. The z-axis gets independent noise for vertical drift.

2. **Sensor model** (inside `update`): Each particle's pose is used to cast rays through the existing occupancy map. The likelihood of the measured ranges given the expected ranges (from raycasting the map) is computed using a Gaussian error model. Only 20 samples per scan are used (subsampled) for performance.

3. **Resampling** (`resample`): Particles are resampled using systematic resampling (low-variance) with weight-proportional selection. This culls low-weight particles and duplicates high-weight ones, concentrating particles in the most likely pose region.

4. **Map update** (`update_map`): The best estimate pose (weighted average of all particles) is used to cast LIDAR rays onto the occupancy grid via `update_ray`. Each hit endpoint marks a cell occupied; all cells along the ray are marked free.

### Why a particle filter?

Particle filters handle non-Gaussian pose distributions (multimodal hypotheses), which can occur when the robot passes through symmetric corridors or ambiguous junctions. An EKF would linearize and could converge to the wrong mode. The tradeoff is computational cost (200 particles × 20 rays = 4000 raycasts per scan).

### Why subsample to 20 rays for sensor model?

Full 180×16 = 2880 rays × 200 particles = 576k raycasts per scan would be too slow. Subsampling to 20 rays per particle (4k raycasts) gives a reasonable likelihood estimate at a fraction of the cost. The map update still uses all rays for map building.

## ROS Bridge

### What

The `cave_robot_node` runs the robot binary as a subprocess and bridges ROS 2 topics to/from it via JSON over stdin/stdout.

### How

`ros/src/cave_robot_node/src/main.rs`:

```
spawn robot binary with --pipe flag and CAVE_FILE env
subscribe to /cave_robot/lidar (sensor_msgs/LaserScan)
  → serialize scan to JSON, write to robot's stdin
read robot's stdout line-by-line (JSON with linear_x, angular_z, ...)
  → deserialize, publish as geometry_msgs/Twist to /model/cave_robot/cmd_vel
```

Topic flow:

```
Gazebo (gpu_lidar) → /cave_robot/lidar ──→ ros_gz_bridge ──→ ROS /cave_robot/lidar
                                                                     ↓
                                                            cave_robot_node subscribes
                                                                     ↓
                                                            serialize to JSON → stdin of robot process
                                                                     ↓
                                                            robot process (--pipe mode)
                                                                     ↓
                                                            stdout ← JSON command
                                                                     ↓
                                                            cave_robot_node publishes
                                                                     ↓
                                                            ROS /model/cave_robot/cmd_vel
                                                                     ↓
                                                            ros_gz_bridge ──→ Gazebo (velocity control)
```

### Why a subprocess bridge?

Running the robot as a separate process isolates the Rust workspace (edition 2024, custom pathfinding/SLAM code) from the ROS 2 build system. No cross-workspace crate dependency, no `ament` Cargo.toml patching. The JSON protocol on pipes makes the robot binary testable outside ROS — you can `echo '{"ranges":...}' | ./robot --pipe`.

## Navigation Loop Detail

### `process_scan` (called each step)

1. Increment step counter
2. Feed scan into SLAM (motion + sensor + resample + map update)
3. Mark current cell as explored
4. If forward mode:
   - Discover obstacles by checking which LIDAR hit endpoints correspond to walls in the ground-truth cave
   - For each newly discovered wall: call `planner.update_obstacle(node, true)`
   - If any new obstacles were found: call `planner.move_start(current_cell)`, then `planner.get_path()` to replan
   - Check if robot reached the end cell
   - `skip_occupied_cells()` to advance past path nodes that are now blocked
   - `move_toward(target_cell)` → compute yaw error, rotate or move forward
5. If return mode: (no sensor, just follow the precomputed A\* path)

### `apply_command`

Applies a simple kinematic model with dt = 0.2s:

```rust
pose.x += linear_x * cos(yaw) * dt
pose.y += linear_x * sin(yaw) * dt
pose.z += linear_z * dt
pose.yaw += angular_z * dt
```

This is a 2.5D model — the robot moves in the horizontal plane based on its heading, with independent vertical velocity. No 6-DOF dynamics: yaw only, no pitch/roll. This matches the Gazebo VelocityControl plugin, which moves the model kinematically.

### `move_toward`

Converts a target grid cell into a NavCommand:
- Compute desired yaw from current position to cell center `(cell.x+0.5, cell.y+0.5)`
- If yaw error > 0.3 rad: rotate in place (angular_z ±0.5)
- Else: drive forward (linear_x = 0.3 if still turning, 0.5 if aligned)

Z-axis movement is not yet implemented — the robot stays at a fixed z-level in the current tests.

### Pipe mode

The `run_pipe()` method reads JSON LIDAR scans from stdin and writes JSON NavCommands to stdout. This is the protocol that `cave_robot_node` uses. The pipe mode never terminates on its own (it would block waiting for stdin EOF), which is appropriate for a ROS node lifecycle.

```
stdin:  {"ranges": [...], "angle_min": ..., "angle_max": ..., ...}
stdout: {"linear_x": 0.5, "linear_y": 0.0, "linear_z": 0.0, "angular_z": 0.0}
```

## Limitations

- **Limited LIDAR FOV**: 180° horizontal × 34° vertical at 8m range discovers very few cells (≈13 out of 6144 in the 64×32×3 cave). This causes the robot to get stuck near the starting area in complex caves because it can't see far enough ahead to plan a path.
- **No vertical pathfinding**: The robot doesn't navigate vertically in practice — it stays on one z-level. The pathfinding supports 26-connected 3D, but `move_toward` and `apply_command` don't drive the z-axis toward a target.
- **SLAM drift**: The particle filter estimate drifts over long distances. The sensor model subsamples to only 20 rays per particle, which provides coarse likelihood discrimination.
- **Forward phase timeout**: After 10,000 steps without reaching the goal, the robot aborts. In the 64×32×3 cave, it gets stuck at step ~200 and times out at 10,000.

## Running

### Offline
```bash
just generate              # Generate cave.json
just robot                 # Run robot binary (reads CAVE_FILE)
CAVE_FILE=./tmp/cave.json timeout 30 cargo run --release -p robot
```

### ROS 2 / Gazebo
```bash
just generate              # Generate cave.json
just generate-world        # Convert to SDF world
just ros-build             # Build ROS packages
just sim                   # Launch Gazebo (GUI)
just bringup               # Full: Gazebo + bridge + robot node
```

See [gazebo.md](gazebo.md) and [setup.md](setup.md) for more details.
