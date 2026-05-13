use super::Map;
use rand::rngs::StdRng;
use rand::RngExt;
use shared::TILE_FLOOR;

impl Map {
    pub(crate) fn pick_start_end(&mut self, rng: &mut StdRng) {
        let mut floors: Vec<(usize, usize, usize)> = Vec::new();
        for level in 0..self.cave.size_z {
            for row in 1..self.cave.size_y - 1 {
                for col in 1..self.cave.size_x - 1 {
                    if self.cave.grid[level][row][col] == TILE_FLOOR {
                        floors.push((col, row, level));
                    }
                }
            }
        }
        if floors.is_empty() {
            eprintln!("warning: no floor tiles found, using default start/end");
            self.cave.start = (1, 1, 0);
            self.cave.end = (1, 1, 0);
            return;
        }

        let index_a = rng.random_range(0..floors.len());
        let mut index_b = rng.random_range(0..floors.len());
        while index_b == index_a && floors.len() > 1 {
            index_b = rng.random_range(0..floors.len());
        }
        self.cave.start = floors[index_a];
        self.cave.end = floors[index_b];
    }
}
