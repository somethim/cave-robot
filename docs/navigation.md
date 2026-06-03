# Navigation

## Overview

Navigation runs in two phases:

| Phase | Route | Path source | Localisation |
|---|---|---|---|
| **Forward** | Start → End | A\* on clearance-weighted known graph | EKF (dead-reckoning + Gazebo pose fusion) |
| **Return** | End → Start | A\* on SLAM-built occupancy map | EKF / Gazebo pose |

The cave map is fully known before navigation starts (loaded from `cave.json`).
The forward phase uses this knowledge to plan an efficient clearance-aware path,
while also building a SLAM occupancy map from LiDAR observations. The return
phase plans on that observed map — what the robot earned by scanning — rather
than the ground-truth grid.

## Coordinate system

All positions are in cave-grid coordinates:
- `x`, `y` in metres (1 cell = 1 m)
- `z` is floor index (integer level), not metres
- `yaw` in radians, wrapped to (−π, π]

Cell centre for grid cell `(cx, cy, cz)`:

```
world_x = cx + 0.5
world_y = cy + 0.5
world_z = cz      (grid-z, not physical metres)
```

Physical Gazebo z = `cz * 2.0 + 0.5 + 0.25` (level spacing + cell-centre bias + spawn offset).

## EKF localisation (`crates/kalman`)

### What

A 4-state Extended Kalman Filter tracks `[x, y, z, yaw]`. It runs throughout
both phases, fusing motor commands with Gazebo pose measurements when available.

### Motion model (predict step)

Called inside `apply_command` on every control step:

```
x'   = x   + v · cos(yaw) · dt
y'   = y   + v · sin(yaw) · dt
z'   = z   + vz · dt
yaw' = wrap(yaw + ω · dt)
```

The state-transition Jacobian `F = I + ∂f/∂x` linearises the nonlinear
heading dependency. Covariance propagates as `P ← F·P·Fᵀ + Q`, where Q is
diagonal and proportional to commanded velocity (faster motion = more process
noise). Yaw is normalised to (−π, π] after each update.

### Measurement update

When a Gazebo pose arrives with the scan (`scan.pose = Some(p)`):

```
for each state dimension i:
    S  = P[i,i] + R[i]          (scalar innovation covariance)
    K  = P[:,i] / S             (Kalman gain column)
    δ  = wrap(z_i − x̂[i])      (innovation, with angle wrap for yaw)
    x̂ += K · δ
    P  -= K · P[i,:]            (Joseph form, dimension-wise)
```

Sequential scalar updates avoid 4×4 matrix inversion. The fused estimate
replaces `self.pose` and feeds into all navigation decisions.

### Without Gazebo pose

The robot falls back to pure dead-reckoning using the EKF predicted state.
Error accumulates over time (the covariance grows), but for the distances
involved in a single cave traverse this is sufficient.

## Forward phase: Start → End

### Path planning

At startup, A\* runs once on the complete graph with clearance-weighted costs:

1. **Wall passability** — impassable cells are marked `cost = ∞`.
2. **Clearance weighting** — BFS flood-fill from all wall cells computes
   Manhattan distance-to-nearest-wall `d` for each passable cell. Cost is set to:
   - `d = 1` (adjacent to wall): 6 ×
   - `d = 2` (one-cell buffer): 2.5 ×
   - `d ≥ 3` (clear): 1 ×

   This routes paths through corridor centres, giving the drone turning margin
   without requiring wider corridors.
3. **Cardinal-only neighbours** — diagonals are excluded to prevent corner clipping.

The resulting path is a sequence of grid cells from start to end. It does not
change unless the drone gets stuck and replanning is triggered.

### Per-step loop

Each LiDAR scan triggers one call to `process_scan`:

1. **EKF predict** — advance EKF from last motor command.
2. **EKF update** — fuse Gazebo pose if present in the scan.
3. **SLAM update** — `slam.update_with_map_pose(scan, self.pose)` traces all
   LiDAR rays from the accurate EKF pose to update the occupancy grid. The
   particle filter is not used for localisation here; only the map write matters.
4. **End detection** — if Euclidean distance to end cell centre < 0.5 m, stop.
5. **Waypoint following** — `move_toward_blind(target)`:
   - Compute heading error = `ang_diff(atan2(dy, dx), yaw)`
   - If `|err| > 0.35 rad`: pure rotation at 1.0 rad/s
   - Else: drive at 0.8 m/s, steering `angular_z = err × 2.5`
6. **Waypoint advance** — path index increments when distance to current target < 0.5 m.

### Recovery

Two stuck detectors share state:

**Spin detection** — if the command has no forward component for 40 consecutive
steps, the drone is spinning in place. Triggers: 12-step backup (−0.4 m/s),
10-step rotate, then A\* replan from current cell.

**No-progress detection** — if the distance to the current waypoint hasn't
decreased for 30 steps, the waypoint is skipped and A\* replans.

## SLAM map building (`crates/slam`)

SLAM runs in parallel with forward navigation and produces the occupancy grid
used for return path planning.

### Occupancy grid

Log-odds representation: each cell stores `l = log(p / (1−p))`. Unknown cells
start at `l = 0` (p = 0.5). Each LiDAR ray update adds:
- `+0.8` to the endpoint (occupied evidence)
- `−0.4` to all intermediate cells (free evidence)

Cells with `p > 0.5` are considered occupied. `to_passable_grid()` thresholds
the full grid to produce a boolean array for A\*.

### Ray tracing

`update_ray(x0,y0,z0, x1,y1,z1)` uses a 3-D Bresenham line to enumerate every
grid cell between origin and endpoint. Intermediate cells are marked free; the
endpoint is marked occupied if it differs from the origin.

### Why the particle filter localisation is bypassed

The `Slam` struct contains a particle filter (200 particles) designed for
simultaneous localisation and mapping. In this implementation `slam.predict()`
is never called, so particles never move from their initialisation around the
spawn position. Calling `slam.estimated_pose()` always returns approximately the
start cell.

The particle filter's *map update* is still useful: `update_with_map_pose`
accepts an explicit pose for ray tracing, bypassing the drifted particle
estimate. This lets the SLAM occupancy grid be built accurately from the
EKF/Gazebo pose, even without functioning localisation in the particle filter.

The result is that the SLAM grid faithfully maps observed free space and walls
while the particle filter side sits idle.

## Return phase: End → Start

### Path planning on SLAM map

When forward is complete, `process_return_scan` initialises the return path:

1. Build a `GridGraph` from `slam.map.to_passable_grid()`.
2. Use `self.pose` (EKF/Gazebo — accurate) as the starting cell.
   **Not** `slam.estimated_pose()` (which is stuck at the spawn position).
3. Run A\* from current cell to start cell on the SLAM graph.
4. Fallback: if SLAM graph has no path (unscanned section), rerun A\* on the
   full known graph.

The SLAM-derived path reflects the robot's observation: corridors it never
scanned appear as unknown (probability 0.5, passable) or occupied depending on
what stray rays marked them. In practice the forward path is well-covered, so
the SLAM path closely mirrors the outbound route in reverse.

### Per-step loop

Identical to forward: EKF predict/update, SLAM update, `move_toward_blind`,
waypoint advance, spin/no-progress recovery.

Start detection: when distance to start cell centre < 0.5 m, `return_done` is
set and the pipe loop exits.

## LiDAR simulation (`crates/shared/src/robot.rs`)

Used in offline mode. Matches the Gazebo GPU LiDAR specification.

| Parameter | Value |
|---|---|
| Horizontal rays | 180 (360° coverage) |
| Vertical rays | 32 (±40° = 80° arc) |
| Max range | 8.0 m |
| Step size | 0.1 m |
| Noise | Gaussian σ = 0.05 m |

`cast_ray_3d` steps 0.1 m along each `(azimuth, elevation)` direction until
a non-passable voxel is hit or max range is reached. The origin z-coordinate is
converted to physical metres for the ray, then back to floor-index for the
occupancy grid.

## Drone motion model

`apply_command(cmd, dt)` integrates kinematic motion at each step:

```rust
pose.x   += cmd.linear_x * cos(pose.yaw) * dt
pose.y   += cmd.linear_x * sin(pose.yaw) * dt
pose.z   += cmd.linear_z * dt
pose.yaw  = wrap(pose.yaw + cmd.angular_z * dt)
```

This is the same model the Gazebo `VelocityControl` plugin applies, so
dead-reckoning and physical simulation stay in sync. `dt = 0.2 s` in offline
mode, measured from the system clock in pipe mode.

## Pipe protocol

In ROS / Gazebo mode the robot binary is spawned with `--pipe`. Scans arrive as
JSON on `stdin`; commands are written as JSON to `stdout`.

```
stdin:   {"ranges":[...], "angle_min":-3.14, "angle_max":3.14,
          "angle_increment":0.035, "angle_min_vertical":0,
          "angle_max_vertical":0, "angle_increment_vertical":0,
          "range_min":0.1, "range_max":8.0, "pose":null}

stdout:  {"linear_x":0.8, "linear_y":0.0, "linear_z":0.0, "angular_z":-0.14}
```

`pose` in the scan is `null` from the ROS node (Gazebo pose publisher reports at
origin due to bridge limitations). The robot uses EKF dead-reckoning. When a
Gazebo pose is available it can be embedded in the JSON and the EKF update fires.

The pipe loop exits (closes stdout) when `return_done` is set. The node detects
stdout close and sets `robot_done`, which stops subsequent scan writes.

## Phase banners

Both modes print human-readable banners at phase transitions:

```
===========================================
Starting
  start:  (13, 2, 0)
  end:    (5, 28, 2)
  planned steps: 22
===========================================

===========================================
End reached, starting return
  end pos:  (5.47, 28.51, 2.00)
  start:    (13, 2, 0)
  planned steps: 21
===========================================
```
