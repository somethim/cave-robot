use super::Map;
use shared::TILE_FLOOR;

impl Map {
    pub(crate) fn flood_fill(&self, level: usize) -> (Vec<Vec<u32>>, u32) {
        let height = self.cave.size_y;
        let width = self.cave.size_x;
        let mut labels = vec![vec![0u32; width]; height];
        let mut next_id = 1u32;
        for row in 1..height - 1 {
            for col in 1..width - 1 {
                if self.cave.grid[level][row][col] == TILE_FLOOR && labels[row][col] == 0 {
                    let mut stack = vec![(col, row)];
                    labels[row][col] = next_id;
                    while let Some((current_col, current_row)) = stack.pop() {
                        for (step_col, step_row) in &[(0isize, -1isize), (0, 1), (-1, 0), (1, 0)] {
                            let neighbor_col = (current_col as isize + step_col) as usize;
                            let neighbor_row = (current_row as isize + step_row) as usize;
                            if neighbor_row < height
                                && neighbor_col < width
                                && self.cave.grid[level][neighbor_row][neighbor_col] == TILE_FLOOR
                                && labels[neighbor_row][neighbor_col] == 0
                            {
                                labels[neighbor_row][neighbor_col] = next_id;
                                stack.push((neighbor_col, neighbor_row));
                            }
                        }
                    }
                    next_id += 1;
                }
            }
        }
        (labels, next_id - 1)
    }
}
