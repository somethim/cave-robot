use super::Map;
use shared::{TILE_FLOOR, TILE_WALL};

impl Map {
    pub(crate) fn step(&mut self, level: usize) {
        let height = self.cave.size_y;
        let width = self.cave.size_x;
        let grid_slice = &self.cave.grid[level];
        let grid2_slice = &mut self.grid2[level];
        for row in 1..height - 1 {
            for col in 1..width - 1 {
                let mut wall_count = 0u32;
                for step_row in -1isize..=1 {
                    for step_col in -1isize..=1 {
                        let nr = (row as isize + step_row) as usize;
                        let nc = (col as isize + step_col) as usize;
                        if grid_slice[nr][nc] != TILE_FLOOR {
                            wall_count += 1;
                        }
                    }
                }
                if wall_count >= self.config.wall_threshold {
                    grid2_slice[row][col] = TILE_WALL;
                } else if wall_count <= self.config.floor_threshold {
                    grid2_slice[row][col] = TILE_FLOOR;
                } else {
                    grid2_slice[row][col] = grid_slice[row][col];
                }
            }
        }
        for row in 1..height - 1 {
            for col in 1..width - 1 {
                self.cave.grid[level][row][col] = self.grid2[level][row][col];
            }
        }
    }
}
