# TODO

Goal: implement the `cave.json -> merged-box Gazebo world -> robot spawn -> ROS bringup` pipeline.

## Phase 1: Package Assets

- [ ] Install robot model assets from `ros/src/cave_robot_description/`.
- [ ] Install world assets from `ros/src/cave_robot_gazebo/`.
- [ ] Add a dedicated generated-assets location for cave worlds and metadata.
- [ ] Update `ros/src/cave_robot_description/CMakeLists.txt` to install `urdf/`.
- [ ] Update `ros/src/cave_robot_gazebo/CMakeLists.txt` to install `worlds/` and any model/resource directories.

Acceptance criteria:

- [ ] `ros2 pkg prefix cave_robot_description` contains the robot SDF/URDF.
- [ ] `ros2 pkg prefix cave_robot_gazebo` contains launch and world/resource files.

## Phase 2: Define Cave-to-World Conventions

- [ ] Lock voxel size in meters.
- [ ] Define world coordinate mapping from `(x, y, z)` grid cells to Gazebo positions.
- [ ] Treat `TILE_WALL` as solid.
- [ ] Treat `TILE_FLOOR`, `TILE_RAMP`, and `TILE_HOLE` as empty navigable space.
- [ ] Define robot spawn pose from `cave.start`.
- [ ] Define a safe spawn height offset above the cell center.

Acceptance criteria:

- [ ] A short doc comment or README section explains scale and coordinate mapping.

## Phase 3: Build the First Procedural World Generator

- [ ] Add a Rust cave-to-world generator near the existing generator code.
- [ ] Read `cave.json` using the shared `Cave` schema from `crates/shared/src/cave.rs`.
- [ ] Convert wall voxels into axis-aligned boxes.
- [ ] Merge neighboring wall voxels into larger boxes to reduce object count.
- [ ] Emit a valid Gazebo `.sdf` or `.world` file.
- [ ] Include ground/light/world boilerplate required by Gazebo Harmonic.
- [ ] Write generated output to a predictable path.
- [ ] Include cave metadata if needed for spawn lookup/debugging.

Implementation notes:

- [ ] Start with horizontal merging on each z-slice.
- [ ] Optionally extend to vertical merging once the simple version works.
- [ ] Keep generation deterministic for a given `cave.json`.

Acceptance criteria:

- [ ] Running the world generator produces a loadable world file.
- [ ] The number of generated collision objects is much lower than one-box-per-wall-voxel.

## Phase 4: Add Gazebo Launch

- [ ] Implement `ros/src/cave_robot_gazebo/launch/gazebo.launch.py`.
- [ ] Pass the generated world path into Gazebo.
- [ ] Export any required Gazebo resource/model search paths.
- [ ] Make `just sim` use the real world file rather than an empty launch stub.

Acceptance criteria:

- [ ] `just sim` opens Gazebo with the generated cave.
- [ ] Gazebo loads without missing-resource errors.

## Phase 5: Spawn the Robot in the Generated Cave

- [ ] Use `ros/src/cave_robot_description/urdf/cave_robot.sdf` as the primary robot spawn asset.
- [ ] Spawn the robot at the converted `cave.start` position.
- [ ] Ensure spawn orientation is sane for first tests.
- [ ] Verify the robot does not start intersecting walls or the floor.

Acceptance criteria:

- [ ] The robot is visible in the generated cave.
- [ ] Gazebo reports no model/plugin load failures.

## Phase 6: Align ROS and Gazebo Topics

- [ ] Audit the current topic mismatch between ROS and Gazebo.
- [ ] Bridge LiDAR output into ROS as `/scan`.
- [ ] Bridge ROS `/cmd_vel` back into Gazebo control.
- [ ] Add any required `ros_gz_bridge` launch actions and runtime dependencies.
- [ ] Confirm the final ROS-facing contract stays:
  - `/scan` for sensor input
  - `/cmd_vel` for velocity output

Known current mismatch:

- [ ] `ros/src/cave_robot_node/src/main.rs` subscribes to `/scan` and publishes `/cmd_vel`.
- [ ] `ros/src/cave_robot_description/urdf/cave_robot.sdf` currently exposes Gazebo-native LiDAR/control topics.

Acceptance criteria:

- [ ] `cave_robot_node` receives live scan data in ROS.
- [ ] A ROS `Twist` command causes movement in Gazebo.

## Phase 7: Implement Bringup

- [ ] Implement `ros/src/cave_robot_bringup/launch/bringup.launch.py`.
- [ ] Launch Gazebo with the generated cave world.
- [ ] Spawn the robot.
- [ ] Start topic bridges.
- [ ] Start `cave_robot_node`.
- [ ] Make `just bringup` the full end-to-end entrypoint.

Acceptance criteria:

- [ ] `just bringup` starts the full stack without manual extra commands.

## Phase 8: Verify End-to-End Behavior

- [ ] Run `just generate` and confirm `cave.json` is produced.
- [ ] Run the cave-to-world generator and confirm a world file is emitted.
- [ ] Run `just sim` and confirm the cave loads.
- [ ] Run `just bringup` and confirm the node starts.
- [ ] Verify LiDAR scans are visible in ROS.
- [ ] Verify the robot moves when `/cmd_vel` is published.
- [ ] Verify the robot starts near `start` and the world reflects the JSON cave structure.

Suggested manual checks:

- [ ] Compare a few slices from `cave.json` against the generated geometry.
- [ ] Check ramp/hole cells for expected open vertical traversal.
- [ ] Check collision behavior in narrow passages.

## Phase 9: Documentation

- [ ] Update `docs/gazebo.md` from planned state to actual implementation steps.
- [ ] Update `docs/setup.md` with real commands for world generation, sim, and bringup.
- [ ] Document how generated cave assets are cleaned/regenerated.
- [ ] Document current limitations of the merged-box world approach.

Acceptance criteria:

- [ ] A new contributor can generate a cave and launch Gazebo by following the docs.

## Phase 10: Optional Mesh Upgrade Later

- [ ] Keep merged-box generation as the baseline procedural pipeline.
- [ ] Evaluate Marching Cubes only after the box-based world is stable.
- [ ] If upgrading, preserve the same `cave.json` input contract and spawn logic.

## First Build Order

- [ ] Install robot/world assets in ROS packages.
- [ ] Implement the merged-box world generator.
- [ ] Implement `gazebo.launch.py`.
- [ ] Spawn `cave_robot.sdf` at `cave.start`.
- [ ] Add ROS/Gazebo topic bridging.
- [ ] Implement `bringup.launch.py`.
- [ ] Run end-to-end verification.
- [ ] Update docs.

## Commands To Use During Development

```bash
just generate
just ros-bootstrap
just ros-build
just sim
just bringup
```

## Nice-to-Have Follow-Ups

- [ ] Add a single command that regenerates `cave.json` and the Gazebo world together.
- [ ] Add a debug view or export summarizing merged box counts.
- [ ] Add a small regression test for cave-to-world generation.
- [ ] Add validation that `start` and `end` are in passable cells before spawning.
