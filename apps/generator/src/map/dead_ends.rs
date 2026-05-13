use super::Map;
use rand::rngs::StdRng;
use rand::RngExt;
use shared::TILE_FLOOR;
use shared::TILE_WALL;

impl Map {
    pub(crate) fn carve_dead_ends(&mut self, level: usize, rng: &mut StdRng, count: usize) {
        let height = self.cave.size_y as i32;
        let width = self.cave.size_x as i32;
        let dirs = [(0i32, -1i32), (0, 1), (-1, 0), (1, 0)];

        for _ in 0..count {
            let mut candidates: Vec<(i32, i32)> = Vec::new();
            for row in 2..height - 2 {
                for col in 2..width - 2 {
                    if self.cave.grid[level][row as usize][col as usize] != TILE_WALL {
                        continue;
                    }
                    if dirs.iter().any(|&(sc, sr)| {
                        let nc = col + sc;
                        let nr = row + sr;
                        self.cave.grid[level][nr as usize][nc as usize] == TILE_FLOOR
                    }) {
                        candidates.push((col, row));
                    }
                }
            }
            if candidates.is_empty() {
                return;
            }

            let (col, row) = candidates[rng.random_range(0..candidates.len())];

            let carve_dir = dirs.iter().find_map(|&(sc, sr)| {
                let nc = col + sc;
                let nr = row + sr;
                if self.cave.grid[level][nr as usize][nc as usize] == TILE_FLOOR {
                    Some((-sc, -sr))
                } else {
                    None
                }
            });

            if let Some((dx, dy)) = carve_dir {
                let length: i32 = rng.random_range(2..=4);
                self.cave.grid[level][row as usize][col as usize] = TILE_FLOOR;
                for i in 1..=length {
                    let nx = col + dx * i;
                    let ny = row + dy * i;
                    if nx < 2 || nx >= width - 2 || ny < 2 || ny >= height - 2 {
                        break;
                    }
                    if self.cave.grid[level][ny as usize][nx as usize] != TILE_WALL {
                        break;
                    }
                    self.cave.grid[level][ny as usize][nx as usize] = TILE_FLOOR;
                }
            }
        }
    }
}
