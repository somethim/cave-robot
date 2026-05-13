# Gazebo Integration

## Current state

The cave generator produces `cave.json` — a 3D voxel grid with wall, floor, ramp,
and hole tiles. The ROS 2 node skeleton subscribes to `/scan` and publishes to
`/cmd_vel`. However, the pipeline from `cave.json` to a simulated Gazebo world
is not yet implemented: `cave_robot_gazebo/` contains empty stubs, there are no
SDF world files, and no mesh conversion code exists.

## Pipeline (target)

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

Fill `cave_robot_gazebo/launch/gazebo.launch.py` and
`cave_robot_bringup/launch/bringup.launch.py` to:

1. Start Gazebo with the generated SDF world
2. Spawn the drone URDF at the cave start position
3. Configure the LIDAR plugin to match the real sensor

### 4. Drone model

A URDF description of the quadrotor with:

- Visual + collision geometry
- IMU and LIDAR sensor plugins
- Ros2 control or direct topic interface (`/cmd_vel`)

## Implementation order

1. **Surface extraction** — add a subcommand or binary to the generator crate
   that writes an STL mesh from `cave.json`
2. **SDF world generator** — write a script that produces a `.world` file
   referencing the mesh
3. **Launch files** — wire up Gazebo + world + drone spawn
4. **Drone URDF** — model the quadrotor with LIDAR
5. **Integration test** — run the full offline → ROS → Gazebo pipeline

## Tile semantics for mesh generation

| Tile  | Gazebo treatment                        |
|-------|-----------------------------------------|
| Wall  | Emit mesh faces; solid collision        |
| Floor | No mesh; traversable surface            |
| Ramp  | No mesh; marked as traversable slope    |
| Hole  | No mesh; vertical passage through floor |
