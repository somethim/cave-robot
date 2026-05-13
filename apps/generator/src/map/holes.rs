use super::Map;
use rand::rngs::StdRng;
use rand::RngExt;
use shared::{TILE_FLOOR, TILE_HOLE, TILE_WALL};

impl Map {
    pub(crate) fn place_holes(&mut self, rng: &mut StdRng, count: usize) {
        let size_z = self.cave.size_z;
        let size_y = self.cave.size_y;
        let size_x = self.cave.size_x;
        let horizontal = [(0isize, -1isize), (0, 1), (-1, 0), (1, 0)];

        for _ in 0..count {
            let mut candidates: Vec<(usize, usize, usize)> = Vec::new();
            for z in 0..size_z - 1 {
                for y in 1..size_y - 1 {
                    for x in 1..size_x - 1 {
                        if self.cave.grid[z][y][x] != TILE_FLOOR {
                            continue;
                        }
                        if self.cave.grid[z + 1][y][x] != TILE_WALL {
                            continue;
                        }
                        let pocket_isolated = horizontal.iter().all(|&(dx, dy)| {
                            let nx = (x as isize + dx) as usize;
                            let ny = (y as isize + dy) as usize;
                            ny >= size_y
                                || nx >= size_x
                                || self.cave.grid[z + 1][ny][nx] == TILE_WALL
                        });
                        if !pocket_isolated {
                            continue;
                        }
                        candidates.push((x, y, z));
                    }
                }
            }
            if candidates.is_empty() {
                return;
            }

            let idx = rng.random_range(0..candidates.len());
            let (hx, hy, hz) = candidates[idx];
            self.cave.grid[hz][hy][hx] = TILE_HOLE;
            self.cave.grid[hz + 1][hy][hx] = TILE_HOLE;
        }
    }
}
