use super::Map;
use shared::{TILE_FLOOR, TILE_WALL};

impl Map {
    pub(crate) fn step(&mut self, level: usize) {
        let height = self.cave.size_y;
        let width = self.cave.size_x;
        for row in 1..height - 1 {
            for col in 1..width - 1 {
                let mut wall_count = 0;
                for step_row in -1isize..=1 {
                    for step_col in -1isize..=1 {
                        let neighbor_row = (row as isize + step_row) as usize;
                        let neighbor_col = (col as isize + step_col) as usize;
                        if self.cave.grid[level][neighbor_row][neighbor_col] != TILE_FLOOR {
                            wall_count += 1;
                        }
                    }
                }
                if wall_count >= self.config.wall_threshold {
                    self.grid2[row][col] = TILE_WALL;
                } else if wall_count <= self.config.floor_threshold {
                    self.grid2[row][col] = TILE_FLOOR;
                } else {
                    self.grid2[row][col] = self.cave.grid[level][row][col];
                }
            }
        }
        for row in 1..height - 1 {
            for col in 1..width - 1 {
                self.cave.grid[level][row][col] = self.grid2[row][col];
            }
        }
    }
}
