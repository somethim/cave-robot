use super::Map;
use shared::{TILE_FLOOR, TILE_WALL};

impl Map {
    pub(crate) fn connect_regions(&mut self, level: usize) {
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
        let max_iterations = 4096;
        for _ in 0..max_iterations {
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
}

fn find_root(parent: &mut [usize], mut node: usize) -> usize {
    while parent[node] != node {
        parent[node] = parent[parent[node]];
        node = parent[node];
    }
    node
}
