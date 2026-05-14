# Cave Generation Algorithm

## Overview

The generator uses a **hybrid 2D→3D approach**: each z-slice is independently
generated as a 2D cave using Cellular Automata with Region Connection
([RogueBasin reference](https://www.roguebasin.com/index.php/Cellular_Automata_Method_for_Generating_Random_Cave-Like_Levels)),
then slices are connected vertically via ramps to form a single 3D traversable
volume.

The cave is a 3D voxel volume of size `size_x × size_y × size_z` (default
64×32×4, configurable via env vars). Each horizontal slice is a proper 2D cave
with winding passages and rooms. Ramps at overlapping floor positions connect
adjacent slices for 3D traversal.

## Pipeline

```
for each z-slice:
    initmap(slice)          → random fill, x/y border walls
    step(slice) × 6         → 2D cellular automaton (3×3 Moore)
    connect_regions(slice)  → 4-directional flood fill + union-find
    carve_dead_ends(slice)  → 8 dead-end tunnels

place_ramps()               → mark overlapping floor as TILE_RAMP
connect_regions_3d()        → 6-directional 3D flood fill + wall breaching
pick_start_end()            → random start/end from entire volume
```

### 1. Random fill (`initmap`, per slice)

For each z-slice, the interior (excluding the solid wall border on all 4 sides)
is initialized with ~50% wall / ~50% floor using a seeded RNG. The border
(x=0, x=max, y=0, y=max) is forced entirely to wall, enclosing the slice.

### 2. Cellular automaton smoothing (`step`, ×6 iterations, per slice)

Each iteration applies a 3×3 Moore neighbourhood rule simultaneously to every
cell:

| Wall neighbours (of 9) | Result    |
|------------------------|-----------|
| ≥ 6                    | Wall      |
| ≤ 3                    | Floor     |
| 4 or 5                 | Unchanged |

Six iterations of this rule carve the random noise into cave-like passages:
walls become contiguous rock, floors form tunnels and caverns.

### 3. Region connection (`connect_regions`, per slice)

After CA smoothing, disconnected floor regions within a single slice may exist.
This phase guarantees a single connected component per slice:

1. **Flood fill** — label each 4-directionally connected floor region with a
   unique ID.
2. **Union-find wall breaching** — scan every wall tile bordering 2+ distinct
   floor regions, convert it to floor, and union the regions.
3. **Manhattan corridor fallback** — if regions remain, find the closest pair
   of floor tiles from different regions and carve an L-shaped corridor between
   them. Repeats until one region remains.

### 4. Dead-end carving (`carve_dead_ends`, per slice)

8 short tunnels are carved from cave edges into wall areas per slice:

1. Collect all wall tiles adjacent to at least one floor tile (the "cave edge").
2. Pick one at random and determine the direction away from the floor (into the
   wall).
3. Carve a straight tunnel of length 2–4 tiles in that direction.

### 5. Ramp placement (`place_ramps`, 3D)

For each adjacent z-slice pair `(z, z+1)`:

1. Collect all `(x, y)` where both slices have `TILE_FLOOR` (overlapping floor).
2. Pick one at random.
3. Set both voxels to `TILE_RAMP`, enabling 3D traversal between the two levels
   at that position.

If no overlap exists (rare), force a random position to floor on both levels.

### 5b. Hole placement (`place_holes`)

After ramps, dead-end vertical shafts are carved:

1. Scan every floor tile on level `z` where the tile directly above `(x, y, z+1)`
   is wall AND all 4 horizontal neighbors of that upper tile are also wall.
2. Convert both the floor tile and the upper pocket to `TILE_HOLE`.

The drone can enter a hole from below and fly up into the pocket, but the
pocket is surrounded by wall — only exit is back down through the hole.
This creates a 3D dead end: vertical traversal that goes nowhere.

`CAVE_HOLE_COUNT` (default 4) controls how many holes are placed across the
entire volume.

### 5c. 3D reconnection (`connect_regions_3d`)

After ramps are placed, a **6-directional 3D flood fill** runs on the entire
volume. Vertical traversal (up/down) is only permitted through `TILE_RAMP`
voxels — normal floor tiles do not connect vertically. A second pass of
union-find wall breaching (6-neighbor) absorbs any orphan ramp islands into
the main component, guaranteeing the full 3D cave is a single connected
component.

### 6. Start/end selection (`pick_start_end`)

Two distinct passable voxels (floor, ramp, or hole) are chosen uniformly at random
from the entire 3D volume as `S` (start) and `E` (end). Since the 3D
reconnection guarantees a single connected component, a path always exists
between them.

## Tile legend

| Char | Meaning         |
|------|-----------------|
| `.`  | Floor           |
| `#`  | Wall            |
| `%`  | Ramp            |
| `O`  | Hole (dead-end) |
| `S`  | Start point     |
| `E`  | End point       |

## Configuration

Generation parameters are set via `.env` at the project root:

```
CAVE_FILE=cave.json
CAVE_FILL_CONFIDENCE=50    # % chance of wall in random init (0–100)
CAVE_WALL_THRESHOLD=6      # 2D: wall neighbours (of 9) ≥ this → wall
CAVE_FLOOR_THRESHOLD=3     # 2D: wall neighbours (of 9) ≤ this → floor
CAVE_SMOOTH_ITERATIONS=6   # CA smoothing passes per slice
CAVE_DEAD_END_COUNT=8      # dead-end tunnels carved per slice
CAVE_HOLE_COUNT=4          # vertical dead-end shafts placed across all levels
```

## JSON export

Both `generator` and `robot` read `CAVE_FILE` from the environment (or
`.env`). The JSON contains the full 3D grid, start, and end — ready for ROS
nodes to read.

```python
import json
cave = json.load(open("cave.json"))
grid = cave["grid"]       # [z][y][x], 0=floor 1=wall 2=ramp 3=hole
start = cave["start"]     # [x, y, z]
end = cave["end"]
```

## Determinism

The same seed always produces the identical cave. This is guaranteed by
`StdRng::seed_from_u64(seed)` with no external entropy sources.

## 3D Pathfinding

D\* Lite and A\* operate on a 26-connected 3D voxel grid. `TILE_RAMP` and
`TILE_HOLE` voxels enable vertical traversal, while `TILE_FLOOR` voxels only
connect horizontally. `TILE_WALL` voxels are impassable. See the `pathfinding`
crate for the implementation.

## Gazebo output

The voxel surface can be converted to a 3D mesh via Marching Cubes or similar
voxel-to-mesh conversion for Gazebo simulation.
