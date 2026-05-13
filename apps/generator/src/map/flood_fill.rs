use super::Map;
use shared::{allows_vertical, is_passable, TILE_FLOOR};

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
                    while let Some((cx, cy)) = stack.pop() {
                        for &(sx, sy) in &[(0isize, -1isize), (0, 1), (-1, 0), (1, 0)] {
                            let nx = (cx as isize + sx) as usize;
                            let ny = (cy as isize + sy) as usize;
                            if ny < height
                                && nx < width
                                && self.cave.grid[level][ny][nx] == TILE_FLOOR
                                && labels[ny][nx] == 0
                            {
                                labels[ny][nx] = next_id;
                                stack.push((nx, ny));
                            }
                        }
                    }
                    next_id += 1;
                }
            }
        }
        (labels, next_id - 1)
    }

    pub(crate) fn flood_fill_3d(&self) -> (Vec<Vec<Vec<u32>>>, u32) {
        let size_z = self.cave.size_z;
        let size_y = self.cave.size_y;
        let size_x = self.cave.size_x;
        let mut labels = vec![vec![vec![0u32; size_x]; size_y]; size_z];
        let mut next_id = 1u32;
        let dirs = [
            (0isize, 0isize, -1isize),
            (0, 0, 1),
            (0, -1, 0),
            (0, 1, 0),
            (-1, 0, 0),
            (1, 0, 0),
        ];

        for z in 0..size_z {
            for y in 0..size_y {
                for x in 0..size_x {
                    if !is_passable(self.cave.grid[z][y][x]) || labels[z][y][x] != 0 {
                        continue;
                    }
                    let mut stack = vec![(x, y, z)];
                    labels[z][y][x] = next_id;
                    while let Some((cx, cy, cz)) = stack.pop() {
                        for &(dx, dy, dz) in &dirs {
                            let nx = (cx as isize + dx) as usize;
                            let ny = (cy as isize + dy) as usize;
                            let nz = (cz as isize + dz) as usize;
                            if ny >= size_y || nx >= size_x || nz >= size_z {
                                continue;
                            }
                            if !is_passable(self.cave.grid[nz][ny][nx]) {
                                continue;
                            }
                            if labels[nz][ny][nx] != 0 {
                                continue;
                            }
                            if dz != 0
                                && !allows_vertical(self.cave.grid[cz][cy][cx])
                                && !allows_vertical(self.cave.grid[nz][ny][nx])
                            {
                                continue;
                            }
                            labels[nz][ny][nx] = next_id;
                            stack.push((nx, ny, nz));
                        }
                    }
                    next_id += 1;
                }
            }
        }
        (labels, next_id - 1)
    }
}
