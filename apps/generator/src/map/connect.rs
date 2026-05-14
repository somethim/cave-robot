use super::Map;
use shared::{is_passable, TILE_FLOOR, TILE_WALL};

impl Map {
    pub(crate) fn connect_regions(&mut self, level: usize) {
        let (labels, region_count) = self.flood_fill(level);
        if region_count <= 1 {
            return;
        }

        let mut parent: Vec<usize> = (0..=region_count as usize).collect();
        let height = self.cave.size_y;
        let width = self.cave.size_x;
        let dirs = [(0isize, -1isize), (0, 1), (-1, 0), (1, 0)];

        for row in 1..height - 1 {
            for col in 1..width - 1 {
                if self.cave.grid[level][row][col] != TILE_WALL {
                    continue;
                }
                let mut neighbor_regions = [0u32; 4];
                let mut distinct_count = 0u32;
                for &(sc, sr) in &dirs {
                    let nc = (col as isize + sc) as usize;
                    let nr = (row as isize + sr) as usize;
                    if nr < height && nc < width && self.cave.grid[level][nr][nc] == TILE_FLOOR {
                        let rid = labels[nr][nc];
                        if rid > 0 && !neighbor_regions[..distinct_count as usize].contains(&rid) {
                            neighbor_regions[distinct_count as usize] = rid;
                            distinct_count += 1;
                        }
                    }
                }
                if distinct_count < 2 {
                    continue;
                }

                let root_a = find_root(&mut parent, neighbor_regions[0] as usize);
                let mut all_same = true;
                for &region in &neighbor_regions[1..distinct_count as usize] {
                    if find_root(&mut parent, region as usize) != root_a {
                        all_same = false;
                        break;
                    }
                }
                if all_same {
                    continue;
                }

                self.cave.grid[level][row][col] = TILE_FLOOR;
                let union_root = find_root(&mut parent, neighbor_regions[0] as usize);
                for &region in &neighbor_regions[0..distinct_count as usize] {
                    let sub = find_root(&mut parent, region as usize);
                    parent[sub] = union_root;
                }
            }
        }

        let max_iterations = 4096;
        for _ in 0..max_iterations {
            let (labels2, count2) = self.flood_fill(level);
            if count2 <= 1 {
                return;
            }

            let mut best = usize::MAX;
            let mut best_a = (0, 0);
            let mut best_b = (0, 0);
            #[allow(clippy::needless_range_loop)]
            for ra in 1..height - 1 {
                for ca in 1..width - 1 {
                    if self.cave.grid[level][ra][ca] != TILE_FLOOR {
                        continue;
                    }
                    let id_a = labels2[ra][ca];
                    for rb in 1..height - 1 {
                        for cb in 1..width - 1 {
                            if self.cave.grid[level][rb][cb] != TILE_FLOOR {
                                continue;
                            }
                            if labels2[rb][cb] == id_a {
                                continue;
                            }
                            let d = ca.abs_diff(cb) + ra.abs_diff(rb);
                            if d < best {
                                best = d;
                                best_a = (ca, ra);
                                best_b = (cb, rb);
                            }
                        }
                    }
                }
            }

            if best < usize::MAX {
                let (ax, ay) = best_a;
                let (bx, by) = best_b;
                for x in ax.min(bx)..=ax.max(bx) {
                    self.cave.grid[level][ay][x] = TILE_FLOOR;
                }
                for y in ay.min(by)..=ay.max(by) {
                    self.cave.grid[level][y][bx] = TILE_FLOOR;
                }
            }
        }
    }

    pub(crate) fn connect_regions_3d(&mut self) {
        let (labels, region_count) = self.flood_fill_3d();
        if region_count <= 1 {
            return;
        }

        let mut parent: Vec<usize> = (0..=region_count as usize).collect();
        let size_z = self.cave.size_z;
        let size_y = self.cave.size_y;
        let size_x = self.cave.size_x;
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
                    if self.cave.grid[z][y][x] != TILE_WALL {
                        continue;
                    }
                    let mut neighbor_regions = [0u32; 6];
                    let mut distinct_count = 0u32;
                    for &(dx, dy, dz) in &dirs {
                        let nx = (x as isize + dx) as usize;
                        let ny = (y as isize + dy) as usize;
                        let nz = (z as isize + dz) as usize;
                        if ny < size_y
                            && nx < size_x
                            && nz < size_z
                            && is_passable(self.cave.grid[nz][ny][nx])
                        {
                            let rid = labels[nz][ny][nx];
                            if rid > 0
                                && !neighbor_regions[..distinct_count as usize].contains(&rid)
                            {
                                neighbor_regions[distinct_count as usize] = rid;
                                distinct_count += 1;
                            }
                        }
                    }
                    if distinct_count < 2 {
                        continue;
                    }

                    let root_a = find_root(&mut parent, neighbor_regions[0] as usize);
                    let mut all_same = true;
                    for &region in &neighbor_regions[1..distinct_count as usize] {
                        if find_root(&mut parent, region as usize) != root_a {
                            all_same = false;
                            break;
                        }
                    }
                    if all_same {
                        continue;
                    }

                    self.cave.grid[z][y][x] = TILE_FLOOR;
                    let union_root = find_root(&mut parent, neighbor_regions[0] as usize);
                    for &region in &neighbor_regions[0..distinct_count as usize] {
                        let sub = find_root(&mut parent, region as usize);
                        parent[sub] = union_root;
                    }
                }
            }
        }
    }
}

fn find_root(parent: &mut [usize], mut node: usize) -> usize {
    while parent[node] != node {
        parent[node] = parent[parent[node]];
        node = parent[node];
    }
    node
}
