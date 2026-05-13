# Cave Generation Algorithm

## Overview

The generator uses a **Cellular Automata with Region Connection** algorithm
([RogueBasin reference](https://www.roguebasin.com/index.php/Cellular_Automata_Method_for_Generating_Random_Cave-Like_Levels))
to produce deterministic multi-floor cave systems from a single seed.

Each floor is 64×32 tiles (configurable in `apps/generator/src/main.rs`), with 4
floors by default. Every floor is guaranteed fully connected internally, and
adjacent floors are linked by staircases.

## Pipeline

```
seed → initmap → step × 6 → connect_regions → carve_dead_ends → next level
                                                                      ↓
                                                            place_stairs
                                                            connect_regions
                                                            pick_start_end
```

### 1. Random fill (`initmap`)

The interior (excluding the solid wall border) is initialized with ~50% wall /
~50% floor using a seeded RNG. The border is forced entirely to wall, enclosing
the level.

### 2. Cellular automaton smoothing (`step`, ×6 iterations)

Each iteration applies a 3×3 Moore neighbourhood rule simultaneously to every
cell:

| Wall neighbours | Result |
|-----------------|--------|
| ≥ 6             | Wall   |
| ≤ 3             | Floor  |
| 4 or 5          | Unchanged |

Six iterations of this rule carve the random noise into cave-like passages:
walls become contiguous rock, floors form tunnels and caverns.

### 3. Region connection (`connect_regions`)

After CA smoothing, disconnected floor regions may exist. This phase guarantees
a single connected component per level:

1. **Flood fill** — label each 4-directionally connected floor region with a
   unique ID.
2. **Union-find wall breaching** — scan every wall tile. If it borders two or
   more *distinct* floor regions (not yet merged), convert it to floor and union
   the regions.
3. **Manhattan corridor fallback** — if regions remain after wall breaching, find
   the closest pair of tiles from different regions and carve an L-shaped
   corridor between them. This repeats (up to 4096 iterations) until only one
   region remains.

### 4. Dead-end carving (`carve_dead_ends`)

After connectivity is established, 8 short tunnels are carved from cave edges
into wall areas:

1. Collect all wall tiles adjacent to at least one floor tile (the "cave edge").
2. Pick one at random and determine the direction away from the floor (into the
   wall).
3. Carve a straight tunnel of length 2–4 tiles in that direction.

This naturally produces two outcomes:
- **Dead ends** — the tunnel terminates inside solid wall.
- **New routes** — the tunnel breaks through to another floor area, creating a
  shortcut or alternate path.

### 5. Stair placement (`place_stairs`)

For each adjacent floor pair `(level, level+1)`:

1. Collect all `(col, row)` where both floors have `TILE_FLOOR`.
2. Pick one at random as the staircase.
3. If no overlap exists (rare), force a random position to floor on both levels.
   This fallback can create orphan 1-tile islands if the chosen tile is surrounded
   by walls.

`stairs[level]` stores the `(col, row)` descending from `level → level+1`.

### 5b. Post-stairs reconnection

After all stairs are placed, `connect_regions` is re-run on every level. This
absorbs any orphan stair islands from the fallback into the main floor component,
guaranteeing the full 3D cave is a single connected component.

### 6. Start/end selection (`pick_start_end`)

Two distinct floor tiles are chosen uniformly at random from the entire 3D
volume as `S` (start) and `E` (end). Since the post-stairs reconnection
guarantees a single connected component across all levels, a path always
exists between them.

## Tile legend

| Char | Meaning       |
|------|---------------|
| `.`  | Floor         |
| `#`  | Wall          |
| `S`  | Start point   |
| `E`  | End point     |
| `D`  | Stairs down   |
| `U`  | Stairs up     |

## Configuration

Generation parameters are set via `.env` at the project root:

```
CAVE_FILE=cave.json
CAVE_FILL_CONFIDENCE=50    # % chance of wall in random init (0–100)
CAVE_WALL_THRESHOLD=6      # wall neighbours ≥ this → becomes wall
CAVE_FLOOR_THRESHOLD=3     # wall neighbours ≤ this → becomes floor
CAVE_SMOOTH_ITERATIONS=6   # CA smoothing passes per floor
CAVE_DEAD_END_COUNT=8      # dead-end tunnels carved per floor
```

## JSON export

Both `generator` and `robot` read `CAVE_FILE` from the environment (or
`.env`). The JSON contains the full 3D grid, start, end, and stairs — ready
for ROS nodes to read.

```python
import json
cave = json.load(open("cave.json"))
grid = cave["grid"]       # [floor][row][col], 0=floor 1=wall
start = cave["start"]     # [col, row, floor]
end = cave["end"]
stairs = cave["stairs"]   # [[col, row], ...] descending from each floor
```

## Determinism

The same seed always produces the identical cave. This is guaranteed by
`StdRng::seed_from_u64(seed)` with no external entropy sources.
