# Cave Generation

## Overview

The generator produces a 3-D voxel cave as a JSON file. Each horizontal z-slice
is generated independently using a cellular automaton, then the slices are
connected vertically through ramps and shafts. A connectivity pass guarantees
the whole cave is reachable from any point.

Default size: 64 × 32 × 4 cells (configurable via env vars). Each cell is 1 m × 1 m
in Gazebo; levels are spaced 2 m apart vertically.

## Pipeline

```
for each z-slice:
    initmap(slice)            → random fill, solid border walls
    step(slice) × 6           → 2-D cellular automaton (3×3 Moore neighbourhood)
    connect_regions(slice)    → flood-fill + union-find wall breaching
    carve_dead_ends(slice)    → short dead-end tunnels off existing edges

place_ramps()                 → TILE_RAMP at overlapping floor positions
place_holes()                 → TILE_HOLE vertical shafts through floor/ceiling
connect_regions_3d()          → 6-directional 3-D flood-fill + wall breaching
pick_start_end()              → random distinct passable cells
```

## Step-by-step

### 1. Random fill (`initmap`)

The interior (excluding the solid border on all four sides) is filled ~50 % wall
/ ~50 % floor using a seeded RNG. The border is forced entirely to wall,
enclosing the slice. The `fill_confidence` parameter controls the wall density.

### 2. Cellular automaton smoothing (`step`, ×6)

Each iteration applies a 3×3 Moore neighbourhood rule to every interior cell
simultaneously (double-buffered):

| Wall neighbours (of 9) | Result |
|---|---|
| ≥ `wall_threshold` (default 6) | Wall |
| ≤ `floor_threshold` (default 3) | Floor |
| 4–5 | Unchanged |

Six passes turn random noise into organic blob shapes — contiguous rock with
winding passages and open caverns.

### 3. Region connection (`connect_regions`)

After smoothing, a slice may contain multiple disconnected floor regions. This
phase guarantees a single connected component:

1. **Flood fill** — 4-directional BFS labels each connected floor region with a
   unique ID.
2. **Union-find wall breaching** — every wall cell that borders two or more
   distinct floor regions is converted to floor. The regions are merged via
   union-find with path compression. Runs in one pass.
3. **Manhattan corridor fallback** — if disconnected regions remain after step 2
   (possible when regions touch diagonally but not cardinally), the algorithm
   finds the closest pair of floor cells across different regions by Manhattan
   distance and carves an L-shaped corridor between them. Repeats up to 4096
   times until one region remains.

### 4. Dead-end carving (`carve_dead_ends`)

Adds exploration interest by punching short tunnels (length 2–4 cells) into
unexplored wall areas:

1. Collect all wall cells adjacent to at least one floor cell (the cave edge).
2. Pick one at random; find the direction pointing away from the existing floor
   (into the wall).
3. Carve a straight tunnel of random length in that direction, stopping if the
   edge is reached or an existing floor cell is hit.

The count is `dead_end_percent % of floor cells per slice` (default 1 %).

### 5. Ramp placement (`place_ramps`)

For each adjacent level pair `(z, z+1)`:

1. Find all `(x, y)` positions where both levels have `TILE_FLOOR` (overlapping
   floor).
2. Pick one at random and mark both voxels as `TILE_RAMP`.

If no overlap exists (rare in narrow caves), a position is forced to floor on
both levels. `TILE_RAMP` enables vertical traversal between the two levels in
both pathfinding and the Gazebo physics.

### 6. Hole placement (`place_holes`)

Vertical dead-end shafts are carved where the geometry allows it: a floor cell
on level `z` whose cell directly above `(x, y, z+1)` is wall, **and** all four
horizontal neighbours of that upper cell are also wall. Both cells are converted
to `TILE_HOLE`. The drone can fly up into the pocket but cannot exit horizontally
— it must descend the same way it entered.

Count is `hole_percent % of total floor cells` (default 1 %).

### 7. 3-D reconnection (`connect_regions_3d`)

After ramps are placed, the full 3-D volume is flood-filled using 6 cardinal
directions. Vertical moves are only permitted through `TILE_RAMP` or `TILE_HOLE`
voxels — `TILE_FLOOR` does not connect vertically. A second union-find wall
breaching pass (identical to the 2-D version, extended to 6 neighbours) absorbs
any disconnected islands. This guarantees the entire cave is reachable from any
passable cell.

### 8. Start/end selection (`pick_start_end`)

Two distinct passable voxels are chosen uniformly at random from the interior of
the full 3-D volume. Because the 3-D reconnection guarantees a single connected
component, a path always exists between them.

## Tile legend

| Char | Constant | Meaning |
|---|---|---|
| `.` | `TILE_FLOOR` | Open passable cell |
| `#` | `TILE_WALL` | Solid impassable wall |
| `%` | `TILE_RAMP` | Passable + vertical connector |
| `O` | `TILE_HOLE` | Passable + vertical shaft (dead-end) |
| `S` | — | Start position |
| `E` | — | End position |

## Configuration

Generation parameters via `.env`:

```
CAVE_FILE=cave.json
CAVE_SIZE_X=64
CAVE_SIZE_Y=32
CAVE_SIZE_Z=4
CAVE_SEED=<integer>               # omit for random seed
CAVE_FILL_CONFIDENCE=50           # % wall in random init (0–100)
CAVE_WALL_THRESHOLD=6             # neighbours ≥ this → wall in CA step
CAVE_FLOOR_THRESHOLD=3            # neighbours ≤ this → floor in CA step
CAVE_SMOOTH_ITERATIONS=6          # CA passes per slice
CAVE_DEAD_END_PERCENT=1           # % of floor tiles → dead-end tunnels per slice
CAVE_HOLE_PERCENT=1               # % of total floor tiles → vertical shafts
```

## JSON format

```json
{
  "grid": [ [ [0,1,0,...], ... ], ... ],   // [z][y][x]: 0=floor 1=wall 2=ramp 3=hole
  "size_x": 64,
  "size_y": 32,
  "size_z": 4,
  "start": [13, 2, 0],                    // [x, y, z]
  "end":   [5, 28, 2]
}
```

## Determinism

The same `CAVE_SEED` always produces identical output. `StdRng::seed_from_u64`
is used with no external entropy sources. Omitting the seed picks a random one
at startup (printed to stderr).
