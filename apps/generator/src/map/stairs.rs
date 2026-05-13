use super::Map;
use rand::rngs::StdRng;
use rand::RngExt;
use shared::TILE_FLOOR;

impl Map {
    pub(crate) fn place_stairs(&mut self, rng: &mut StdRng) {
        let height = self.cave.size_y;
        let width = self.cave.size_x;
        for level in 0..self.cave.size_z - 1 {
            let mut candidates: Vec<(usize, usize)> = Vec::new();
            for row in 1..height - 1 {
                for col in 1..width - 1 {
                    if self.cave.grid[level][row][col] == TILE_FLOOR
                        && self.cave.grid[level + 1][row][col] == TILE_FLOOR
                    {
                        candidates.push((col, row));
                    }
                }
            }
            if candidates.is_empty() {
                let stair_col = rng.random_range(1..width - 1);
                let stair_row = rng.random_range(1..height - 1);
                self.cave.grid[level][stair_row][stair_col] = TILE_FLOOR;
                self.cave.grid[level + 1][stair_row][stair_col] = TILE_FLOOR;
                self.cave.stairs.push((stair_col, stair_row));
            } else {
                let chosen = rng.random_range(0..candidates.len());
                self.cave.stairs.push(candidates[chosen]);
            }
        }
    }
}
