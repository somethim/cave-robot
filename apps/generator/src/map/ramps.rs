use super::Map;
use rand::rngs::StdRng;
use rand::RngExt;
use shared::{TILE_FLOOR, TILE_RAMP};

impl Map {
    pub(crate) fn place_ramps(&mut self, rng: &mut StdRng) {
        let size_z = self.cave.size_z;
        let size_y = self.cave.size_y;
        let size_x = self.cave.size_x;
        for level in 0..size_z - 1 {
            let mut candidates: Vec<(usize, usize)> = Vec::new();
            for y in 1..size_y - 1 {
                for x in 1..size_x - 1 {
                    if self.cave.grid[level][y][x] == TILE_FLOOR
                        && self.cave.grid[level + 1][y][x] == TILE_FLOOR
                    {
                        candidates.push((x, y));
                    }
                }
            }
            let (rx, ry) = if candidates.is_empty() {
                let fx = rng.random_range(1..size_x - 1);
                let fy = rng.random_range(1..size_y - 1);
                self.cave.grid[level][fy][fx] = TILE_FLOOR;
                self.cave.grid[level + 1][fy][fx] = TILE_FLOOR;
                (fx, fy)
            } else {
                candidates[rng.random_range(0..candidates.len())]
            };
            self.cave.grid[level][ry][rx] = TILE_RAMP;
            self.cave.grid[level + 1][ry][rx] = TILE_RAMP;
        }
    }
}
