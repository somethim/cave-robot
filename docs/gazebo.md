# Gazebo Integration

## Current state

The cave generator produces `cave.json` — a 3D voxel grid with wall, floor, ramp,
and hole tiles. The repository now generates a Gazebo world directly from that
JSON, embeds the robot model at `cave.start`, and launches the result with
`just sim`. The ROS topic bridge and full `bringup` path are still under
construction.

## World conventions

These conventions are fixed for the first procedural Gazebo pipeline and are
implemented in `crates/shared/src/cave.rs`.

### Tile semantics

| Tile  | Gazebo treatment                     |
|-------|--------------------------------------|
| Wall  | Solid wall geometry and collision    |
| Floor | Floor slab                           |
| Ramp  | Open vertical passage + ramp visual  |
| Hole  | Open vertical passage + shaft visual |

The generated world currently includes:

- merged wall boxes
- floor slabs for non-vertical passable cells
- visible ramp connector geometry
- visible hole shaft geometry
- an embedded robot spawned at `cave.start`
- inspection lights for each floor level

### Scale

- `1` voxel = `1.0 m` in Gazebo
- Generated levels are vertically separated by `3.0 m` for inspection
- Cell centers are mapped to `(x + 0.5, y + 0.5)` in XY and to the center of
  the spaced level in Z
- The initial robot spawn pose uses the cell center of `cave.start`
- The robot spawn gets an additional `0.25 m` Z offset to avoid starting in
  contact with cave geometry

### Coordinate mapping

For a cave grid cell `(x, y, z)`:

```text
world_x = (x + 0.5) * 1.0
world_y = (y + 0.5) * 1.0
world_z = z * 3.0 + 0.5
```

Robot spawn position:

```text
spawn_x = world_x
spawn_y = world_y
spawn_z = world_z + 0.25
```

### Asset layout

- Robot description assets install under
  `share/cave_robot_description/urdf/`
- Gazebo launch files install under
  `share/cave_robot_gazebo/launch/`
- Gazebo world assets install under
  `share/cave_robot_gazebo/worlds/`
- Generated cave worlds should be written under
  `share/cave_robot_gazebo/worlds/generated/` in the installed layout, or the
  matching source-tree path during development

## First procedural generator

The current world generator reads `cave.json` and writes a Gazebo SDF world for
visual inspection and upcoming robot integration.

- Input: `CAVE_FILE`
- Output: `CAVE_WORLD_FILE`
- Robot model source: `CAVE_ROBOT_SDF_FILE`
- Sidecar metadata: `<world name>.meta.json`

Current merge strategy:

- Merge contiguous `TILE_WALL` cells into rectangles within each z-slice
- Emit one SDF box per merged rectangle
- Emit merged floor slabs for non-vertical passable cells
- Add deterministic ramp visuals for `TILE_RAMP`
- Add shaft visuals for `TILE_HOLE`
- Group cave geometry into one static cave link to reduce Gazebo load time

Run it with:

```bash
just generate
just generate-world
```

## Launching Gazebo

`ros/src/cave_robot_gazebo/launch/gazebo.launch.py` now launches `gz sim`
against the generated world.

World resolution order:

1. `CAVE_WORLD_FILE` if set in the environment
2. `ros/src/cave_robot_gazebo/worlds/generated/cave.world` in the workspace
3. the installed world under `share/cave_robot_gazebo/worlds/generated/`

Typical flow:

```bash
just generate
just generate-world
just ros-build
just sim
```

Notes:

- `just sim` defaults to GUI mode and forces X11/XCB rather than Wayland for
  better Gazebo compatibility inside the distrobox.
- `just sim-dev` regenerates the cave JSON, rebuilds the Gazebo world, rebuilds
  ROS packages, and launches Gazebo in one step.
- If GUI rendering is still broken on a given machine, headless fallback is:

```bash
ros2 launch cave_robot_gazebo gazebo.launch.py headless:=true
```

## Pipeline (next)

```
generator → cave.json ─→ mesh (STL/COLLADA) ─→ SDF world → Gazebo
                          ↗                          ↓
                   Marching Cubes              drone URDF + LIDAR plugin
                   or surface extraction
```

### 1. Voxel-to-mesh conversion

Two approaches:

**A. Surface extraction (simpler)** — for each wall voxel adjacent to a
non-wall voxel (floor, ramp, hole, or void), emit one or more quads facing
that direction. This produces a closed, manifold mesh of the cave walls with
no interior geometry.

**B. Marching Cubes (smoother)** — process the voxel grid with a standard
Marching Cubes algorithm. Produces smoother, more natural-looking cave walls
at the cost of implementation complexity.

Either approach should output a standard mesh format Gazebo can load:

| Format           | Pros                     | Cons                       |
|------------------|--------------------------|----------------------------|
| STL (`.stl`)     | Widely supported, simple | No colour, binary or ASCII |
| COLLADA (`.dae`) | Supports colour/metadata | More complex format        |
| OBJ (`.obj`)     | Simple, widely supported | Separate material file     |

### 2. SDF world generation

A small script (Rust or Python) reads `cave.json` and the generated mesh, then
writes a `.sdf` world file:

```xml

<sdf version="1.7">
    <world name="cave">
        <include>
            <uri>model://sun</uri>
        </include>
        <model name="cave_walls">
            <static>true</static>
            <link name="walls">
                <collision name="walls">
                    <geometry>
                        <mesh>
                            <uri>model://cave_walls/mesh.dae</uri>
                        </mesh>
                    </geometry>
                </collision>
                <visual name="walls">
                    <geometry>
                        <mesh>
                            <uri>model://cave_walls/mesh.dae</uri>
                        </mesh>
                    </geometry>
                </visual>
            </link>
        </model>
        <!-- drone model with LIDAR plugin -->
    </world>
</sdf>
```

### 3. Launch files

`cave_robot_gazebo/launch/gazebo.launch.py` already starts Gazebo with the
generated world. The next launch task is filling
`cave_robot_bringup/launch/bringup.launch.py` to start bridges and the ROS node.

### 4. Drone model

The quadrotor model exists in `cave_robot_description/urdf/` and is embedded in
the generated world. Remaining work is on bridge/control integration rather than
basic model loading.

## Implementation order

1. **ROS/Gazebo bridge** — wire Gazebo LiDAR and velocity control to ROS topics
2. **Bringup launch** — start Gazebo + bridges + `cave_robot_node`
3. **Integration test** — run the full offline → ROS → Gazebo pipeline
4. **Optional mesh upgrade** — replace block/slab cave visuals with a smoother
   mesh path later

## Tile semantics for mesh generation

| Tile  | Gazebo treatment                        |
|-------|-----------------------------------------|
| Wall  | Emit mesh faces; solid collision        |
| Floor | No mesh; traversable surface            |
| Ramp  | No mesh; marked as traversable slope    |
| Hole  | No mesh; vertical passage through floor |
