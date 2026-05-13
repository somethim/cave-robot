use super::Map;
use rand::rngs::StdRng;
use rand::RngExt as _;
use shared::{TILE_FLOOR, TILE_WALL};

impl Map {
    fn randpick(&self, rng: &mut StdRng) -> u8 {
        if rng.random::<u32>() % 100 < self.config.fill_confidence {
            TILE_WALL
        } else {
            TILE_FLOOR
        }
    }

    pub(crate) fn initmap(&mut self, level: usize, rng: &mut StdRng) {
        let height = self.cave.size_y;
        let width = self.cave.size_x;
        for row in 1..height - 1 {
            for col in 1..width - 1 {
                self.cave.grid[level][row][col] = self.randpick(rng);
            }
        }

        for row in 0..height {
            for col in 0..width {
                self.grid2[level][row][col] = TILE_WALL;
            }
        }

        for row in 0..height {
            self.cave.grid[level][row][0] = TILE_WALL;
            self.cave.grid[level][row][width - 1] = TILE_WALL;
        }

        for col in 0..width {
            self.cave.grid[level][0][col] = TILE_WALL;
            self.cave.grid[level][height - 1][col] = TILE_WALL;
        }
    }
}
