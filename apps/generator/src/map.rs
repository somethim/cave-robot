use rand::rngs::StdRng;
use rand::RngExt;
use rand::SeedableRng;
use shared::{Cave, TILE_FLOOR, TILE_WALL};

#[derive(Clone, Copy)]
pub struct GeneratorConfig {
    pub fill_confidence: u32,
    pub wall_threshold: u32,
    pub floor_threshold: u32,
    pub smooth_iterations: usize,
    pub dead_end_count: usize,
}

impl Default for GeneratorConfig {
    fn default() -> Self {
        Self {
            fill_confidence: 50,
            wall_threshold: 6,
            floor_threshold: 3,
            smooth_iterations: 6,
            dead_end_count: 8,
        }
    }
}

pub struct Map {
    pub cave: Cave,
    grid2: Vec<Vec<u8>>,
    config: GeneratorConfig,
}

impl Map {
    pub fn with_config(
        size_x: usize,
        size_y: usize,
        size_z: usize,
        config: GeneratorConfig,
    ) -> Self {
        Self {
            cave: Cave::new(size_x, size_y, size_z),
            grid2: vec![vec![TILE_FLOOR; size_x]; size_y],
            config,
        }
    }

    fn randpick(&self, rng: &mut StdRng) -> u8 {
        if rng.random::<u32>() % 100 < self.config.fill_confidence {
            TILE_WALL
        } else {
            TILE_FLOOR
        }
    }

    fn initmap(&mut self, level: usize, rng: &mut StdRng) {
        let height = self.cave.size_y;
        let width = self.cave.size_x;
        for row in 1..height - 1 {
            for col in 1..width - 1 {
                self.cave.grid[level][row][col] = self.randpick(rng);
            }
        }

        for row in 0..height {
            for col in 0..width {
                self.grid2[row][col] = TILE_WALL;
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

    fn step(&mut self, level: usize) {
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

    fn flood_fill(&self, level: usize) -> (Vec<Vec<u32>>, u32) {
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

    fn connect_regions(&mut self, level: usize) {
        let (labels, region_count) = self.flood_fill(level);
        if region_count <= 1 {
            return;
        }

        let mut parent: Vec<usize> = (0..=region_count as usize).collect();

        let height = self.cave.size_y;
        let width = self.cave.size_x;
        for row in 1..height - 1 {
            for col in 1..width - 1 {
                if self.cave.grid[level][row][col] != TILE_WALL {
                    continue;
                }
                let directions = [(0isize, -1isize), (0, 1), (-1, 0), (1, 0)];
                let mut neighbor_regions = [0u32; 4];
                let mut distinct_count = 0u32;
                for (step_col, step_row) in &directions {
                    let neighbor_col = (col as isize + step_col) as usize;
                    let neighbor_row = (row as isize + step_row) as usize;
                    if neighbor_row < height
                        && neighbor_col < width
                        && self.cave.grid[level][neighbor_row][neighbor_col] == TILE_FLOOR
                    {
                        let region_id = labels[neighbor_row][neighbor_col];
                        if region_id > 0
                            && !neighbor_regions[..distinct_count as usize].contains(&region_id)
                        {
                            neighbor_regions[distinct_count as usize] = region_id;
                            distinct_count += 1;
                        }
                    }
                }
                if distinct_count < 2 {
                    continue;
                }

                let root_a = find_root(&mut parent, neighbor_regions[0] as usize);
                let mut all_same_component = true;
                for index in 1..distinct_count as usize {
                    if find_root(&mut parent, neighbor_regions[index] as usize) != root_a {
                        all_same_component = false;
                        break;
                    }
                }
                if all_same_component {
                    continue;
                }

                self.cave.grid[level][row][col] = TILE_FLOOR;
                let union_root = find_root(&mut parent, neighbor_regions[0] as usize);
                for index in 0..distinct_count as usize {
                    let subroot = find_root(&mut parent, neighbor_regions[index] as usize);
                    parent[subroot] = union_root;
                }
            }
        }

        // Manhattan corridor fallback — repeat until all regions are connected
        loop {
            let (labels_after, region_count_after) = self.flood_fill(level);
            if region_count_after <= 1 {
                return;
            }

            let mut shortest_distance = usize::MAX;
            let mut best_pair = ((0, 0), (0, 0));
            for search_row_a in 1..height - 1 {
                for search_col_a in 1..width - 1 {
                    if self.cave.grid[level][search_row_a][search_col_a] != TILE_FLOOR {
                        continue;
                    }
                    let region_id_a = labels_after[search_row_a][search_col_a] as usize;
                    for search_row_b in 1..height - 1 {
                        for search_col_b in 1..width - 1 {
                            if self.cave.grid[level][search_row_b][search_col_b] != TILE_FLOOR {
                                continue;
                            }
                            if labels_after[search_row_b][search_col_b] == region_id_a as u32 {
                                continue;
                            }
                            let distance = search_col_a.abs_diff(search_col_b)
                                + search_row_a.abs_diff(search_row_b);
                            if distance < shortest_distance {
                                shortest_distance = distance;
                                best_pair =
                                    ((search_col_a, search_row_a), (search_col_b, search_row_b));
                            }
                        }
                    }
                }
            }
            if shortest_distance < usize::MAX {
                let ((col_a, row_a), (col_b, row_b)) = best_pair;
                for col in col_a.min(col_b)..=col_a.max(col_b) {
                    self.cave.grid[level][row_a][col] = TILE_FLOOR;
                }
                for row in row_a.min(row_b)..=row_a.max(row_b) {
                    self.cave.grid[level][row][col_b] = TILE_FLOOR;
                }
            }
        }
    }

    fn place_stairs(&mut self, rng: &mut StdRng) {
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

    fn carve_dead_ends(&mut self, level: usize, rng: &mut StdRng, count: usize) {
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

    fn pick_start_end(&mut self, rng: &mut StdRng) {
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

    pub fn generate(&mut self, seed: u64) {
        let mut rng = StdRng::seed_from_u64(seed);
        for level in 0..self.cave.size_z {
            self.initmap(level, &mut rng);
            for _ in 0..self.config.smooth_iterations {
                self.step(level);
            }
            self.connect_regions(level);
            self.carve_dead_ends(level, &mut rng, self.config.dead_end_count);
        }
        self.place_stairs(&mut rng);
        self.pick_start_end(&mut rng);
    }
}

fn find_root(parent: &mut [usize], mut node: usize) -> usize {
    while parent[node] != node {
        parent[node] = parent[parent[node]];
        node = parent[node];
    }
    node
}
