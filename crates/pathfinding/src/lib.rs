use std::cmp::Ordering;
use std::collections::BinaryHeap;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Node(pub usize, pub usize, pub usize);

impl Node {
    /// Cardinal-only neighbours (±x, ±y, ±z — no diagonals).
    /// Diagonal moves look shorter on the grid but require the drone to squeeze
    /// through wall corners at 45°, which its physical body cannot reliably do.
    pub fn neighbors(&self, width: usize, height: usize, depth: usize) -> Vec<Node> {
        let (x, y, z) = (self.0, self.1, self.2);
        let mut ns = Vec::with_capacity(6);
        for (dx, dy, dz) in [
            (1i64, 0i64, 0i64), (-1, 0, 0),
            (0, 1, 0),          (0, -1, 0),
            (0, 0, 1),          (0, 0, -1),
        ] {
            let nx = x as i64 + dx;
            let ny = y as i64 + dy;
            let nz = z as i64 + dz;
            if nx >= 0 && nx < width as i64
                && ny >= 0 && ny < height as i64
                && nz >= 0 && nz < depth as i64
            {
                ns.push(Node(nx as usize, ny as usize, nz as usize));
            }
        }
        ns
    }
}

#[derive(Clone)]
pub struct GridGraph {
    pub width: usize,
    pub height: usize,
    pub depth: usize,
    costs: Vec<f64>,
}

impl GridGraph {
    pub fn new(width: usize, height: usize, depth: usize) -> Self {
        Self {
            width,
            height,
            depth,
            costs: vec![1.0; width * height * depth],
        }
    }

    fn idx(&self, n: Node) -> usize {
        n.2 * self.width * self.height + n.1 * self.width + n.0
    }

    fn in_bounds(&self, n: Node) -> bool {
        n.0 < self.width && n.1 < self.height && n.2 < self.depth
    }

    pub fn set_passable(&mut self, n: Node, passable: bool) {
        if self.in_bounds(n) {
            let i = self.idx(n);
            self.costs[i] = if passable { 1.0 } else { f64::INFINITY };
        }
    }

    pub fn is_passable(&self, n: Node) -> bool {
        if !self.in_bounds(n) {
            return false;
        }
        self.costs[self.idx(n)].is_finite()
    }

    pub fn cost(&self, n: Node) -> f64 {
        if !self.in_bounds(n) {
            return f64::INFINITY;
        }
        self.costs[self.idx(n)]
    }

    pub fn set_cost(&mut self, n: Node, cost: f64) {
        if self.in_bounds(n) {
            let i = self.idx(n);
            self.costs[i] = cost;
        }
    }

    pub fn neighbors(&self, n: Node) -> Vec<(Node, f64)> {
        n.neighbors(self.width, self.height, self.depth)
            .into_iter()
            .filter(|nb| self.is_passable(*nb))
            .map(|nb| (nb, self.cost(nb)))
            .collect()
    }

    pub fn heuristic(&self, a: Node, b: Node) -> f64 {
        octile_distance_3d(a, b)
    }

    pub fn from_passable_grid(grid: &[Vec<Vec<bool>>]) -> Self {
        let depth = grid.len();
        let height = grid[0].len();
        let width = grid[0][0].len();
        let mut g = Self::new(width, height, depth);
        for (z, layer) in grid.iter().enumerate() {
            for (y, row) in layer.iter().enumerate() {
                for (x, &passable) in row.iter().enumerate() {
                    if !passable {
                        g.set_passable(Node(x, y, z), false);
                    }
                }
            }
        }
        g
    }
}

// Kept for D* Lite pred_of — with cardinal-only neighbours this always returns
// true, but the check is harmless and guards against accidental diagonal edges.
fn is_direct_move_valid(from: Node, to: Node, graph: &GridGraph) -> bool {
    let dx = to.0 as i64 - from.0 as i64;
    let dy = to.1 as i64 - from.1 as i64;
    let dz = to.2 as i64 - from.2 as i64;
    if dx.abs() > 1 || dy.abs() > 1 || dz.abs() > 1 {
        return false;
    }
    let axes = (dx != 0) as u32 + (dy != 0) as u32 + (dz != 0) as u32;
    if axes <= 1 {
        return true;
    }
    (dx == 0 || graph.is_passable(Node(from.0.wrapping_add_signed(dx as isize), from.1, from.2)))
        && (dy == 0 || graph.is_passable(Node(from.0, from.1.wrapping_add_signed(dy as isize), from.2)))
        && (dz == 0 || graph.is_passable(Node(from.0, from.1, from.2.wrapping_add_signed(dz as isize))))
}

fn euclidean_distance(a: Node, b: Node) -> f64 {
    let dx = a.0 as f64 - b.0 as f64;
    let dy = a.1 as f64 - b.1 as f64;
    let dz = a.2 as f64 - b.2 as f64;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn octile_distance_3d(a: Node, b: Node) -> f64 {
    let dx = (a.0 as i64 - b.0 as i64).unsigned_abs() as f64;
    let dy = (a.1 as i64 - b.1 as i64).unsigned_abs() as f64;
    let dz = (a.2 as i64 - b.2 as i64).unsigned_abs() as f64;
    let (mut d1, mut d2, mut d3) = (dx, dy, dz);
    if d1 < d2 {
        std::mem::swap(&mut d1, &mut d2);
    }
    if d1 < d3 {
        std::mem::swap(&mut d1, &mut d3);
    }
    if d2 < d3 {
        std::mem::swap(&mut d2, &mut d3);
    }
    (d2 - d3) * 2.0_f64.sqrt() + (d1 - d2) * 1.0 + d3 * 3.0_f64.sqrt()
}

#[derive(Clone, Debug)]
struct AStarNode {
    node: Node,
    f: f64,
    g: f64,
}

impl Eq for AStarNode {}

impl PartialEq for AStarNode {
    fn eq(&self, other: &Self) -> bool {
        self.node == other.node && self.f == other.f
    }
}

impl Ord for AStarNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f
            .partial_cmp(&self.f)
            .unwrap_or(Ordering::Equal)
            .then_with(|| self.node.0.cmp(&other.node.0))
    }
}

impl PartialOrd for AStarNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub fn astar(
    graph: &GridGraph,
    start: Node,
    goal: Node,
) -> Option<(Vec<Node>, f64)> {
    let mut open = BinaryHeap::new();
    let mut g_scores: Vec<f64> = vec![f64::INFINITY; graph.width * graph.height * graph.depth];
    let mut came_from: Vec<Option<Node>> =
        vec![None; graph.width * graph.height * graph.depth];

    let sidx = graph.idx(start);
    g_scores[sidx] = 0.0;
    open.push(AStarNode {
        node: start,
        f: graph.heuristic(start, goal),
        g: 0.0,
    });

    while let Some(current) = open.pop() {
        let cidx = graph.idx(current.node);
        if g_scores[cidx] < current.g {
            continue;
        }

        if current.node == goal {
            let mut path = Vec::new();
            let mut n = current.node;
            loop {
                path.push(n);
                let idx = graph.idx(n);
                match came_from[idx] {
                    Some(parent) => n = parent,
                    None => break,
                }
            }
            path.reverse();
            return Some((path, g_scores[cidx]));
        }

        for (neighbor, edge_cost) in graph.neighbors(current.node) {
            let tentative_g = current.g + edge_cost;
            let nidx = graph.idx(neighbor);
            if tentative_g < g_scores[nidx] {
                g_scores[nidx] = tentative_g;
                came_from[nidx] = Some(current.node);
                let f = tentative_g + graph.heuristic(neighbor, goal);
                open.push(AStarNode {
                    node: neighbor,
                    f,
                    g: tentative_g,
                });
            }
        }
    }

    None
}

#[derive(Clone, Debug, PartialEq)]
struct Key {
    k1: f64,
    k2: f64,
}

impl Eq for Key {}

impl PartialOrd for Key {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Key {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .k1
            .partial_cmp(&self.k1)
            .unwrap_or(Ordering::Equal)
            .then_with(|| other.k2.partial_cmp(&self.k2).unwrap_or(Ordering::Equal))
    }
}

#[derive(Clone, Debug)]
struct QueueEntry {
    node: Node,
    key: Key,
}

impl Eq for QueueEntry {}

impl PartialEq for QueueEntry {
    fn eq(&self, other: &Self) -> bool {
        self.node == other.node && self.key == other.key
    }
}

impl Ord for QueueEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        self.key.cmp(&other.key)
    }
}

impl PartialOrd for QueueEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub struct DStarLite {
    graph: GridGraph,
    start: Node,
    goal: Node,
    g: Vec<f64>,
    rhs: Vec<f64>,
    km: f64,
    queue: BinaryHeap<QueueEntry>,
    keys: Vec<Option<Key>>,
}

impl DStarLite {
    pub fn new(graph: GridGraph, start: Node, goal: Node) -> Self {
        let size = graph.width * graph.height * graph.depth;
        let g = vec![f64::INFINITY; size];
        let mut rhs = vec![f64::INFINITY; size];

        let gidx = graph.idx(goal);
        rhs[gidx] = 0.0;

        let km = 0.0;
        let keys = vec![None; size];

        let mut slf = Self {
            graph,
            start,
            goal,
            g,
            rhs,
            km,
            queue: BinaryHeap::new(),
            keys,
        };

        slf.queue.push(QueueEntry {
            node: goal,
            key: slf.calculate_key(goal),
        });
        slf.keys[slf.graph.idx(goal)] = slf.queue.peek().map(|e| e.key.clone());

        slf
    }

    fn calculate_key(&self, s: Node) -> Key {
        let h = self.graph.heuristic(self.start, s);
        let min_grhs = self.g[self.graph.idx(s)].min(self.rhs[self.graph.idx(s)]);
        Key {
            k1: min_grhs + h + self.km,
            k2: min_grhs,
        }
    }

    fn top_key(&mut self) -> Option<Key> {
        loop {
            let entry = self.queue.peek()?;
            let sidx = self.graph.idx(entry.node);
            let stored = &self.keys[sidx];
            match stored {
                Some(k) if *k == entry.key => return Some(entry.key.clone()),
                _ => {
                    self.queue.pop();
                    continue;
                }
            }
        }
    }

    fn update_vertex(&mut self, u: Node) {
        let uidx = self.graph.idx(u);
        if u != self.goal {
            let mut min_rhs = f64::INFINITY;
            for (sp, cost) in self.graph.neighbors(u) {
                let val = cost + self.g[self.graph.idx(sp)];
                if val < min_rhs {
                    min_rhs = val;
                }
            }
            self.rhs[uidx] = min_rhs;
        }

        if let Some(ref old_key) = self.keys[uidx]
            && (old_key != &self.calculate_key(u) || self.g[uidx] != self.rhs[uidx])
        {
            self.keys[uidx] = None;
        }

        if self.g[uidx] != self.rhs[uidx] {
            if self.keys[uidx].is_none() {
                let key = self.calculate_key(u);
                self.keys[uidx] = Some(key.clone());
                self.queue.push(QueueEntry { node: u, key });
            }
        } else {
            self.keys[uidx] = None;
        }
    }

    pub fn compute_shortest_path(&mut self) {
        loop {
            let top = match self.top_key() {
                Some(k) => k,
                None => return,
            };
            let start_key = self.calculate_key(self.start);
            let start_consistent = self.rhs[self.graph.idx(self.start)]
                == self.g[self.graph.idx(self.start)];
            let top_ge_start = top.k1 > start_key.k1
                || (top.k1 == start_key.k1 && top.k2 >= start_key.k2);
            if top_ge_start && start_consistent {
                return;
            }

            let u = match self.queue.pop() {
                Some(e) => {
                    self.keys[self.graph.idx(e.node)] = None;
                    e.node
                }
                None => return,
            };
            let uidx = self.graph.idx(u);

            if self.g[uidx] > self.rhs[uidx] {
                self.g[uidx] = self.rhs[uidx];
                for (pred, _) in self.pred_of(u) {
                    self.update_vertex(pred);
                }
            } else {
                self.g[uidx] = f64::INFINITY;
                for (pred, _) in self.pred_of(u) {
                    self.update_vertex(pred);
                }
                self.update_vertex(u);
            }
        }
    }

    fn pred_of(&self, n: Node) -> Vec<(Node, f64)> {
        n.neighbors(self.graph.width, self.graph.height, self.graph.depth)
            .into_iter()
            .filter(|nb| self.graph.is_passable(*nb))
            .filter(|nb| is_direct_move_valid(*nb, n, &self.graph))
            .map(|nb| {
                let cost = self.graph.cost(nb);
                let dist = euclidean_distance(n, nb);
                (nb, cost * dist)
            })
            .collect()
    }

    pub fn get_path(&mut self) -> Option<Vec<Node>> {
        self.compute_shortest_path();

        if self.g[self.graph.idx(self.start)] == f64::INFINITY {
            return None;
        }

        let mut path = Vec::new();
        let mut current = self.start;

        while current != self.goal {
            path.push(current);
            let mut best = f64::INFINITY;
            let mut next = None;

            for (neighbor, cost) in self.graph.neighbors(current) {
                let nidx = self.graph.idx(neighbor);
                let val = cost + self.g[nidx];
                if val < best {
                    best = val;
                    next = Some(neighbor);
                }
            }

            match next {
                Some(n) => current = n,
                None => {
                    if !path.is_empty() && path[path.len() - 1] == current {
                        break;
                    }
                    return None;
                }
            }
        }
        path.push(self.goal);

        Some(path)
    }

    pub fn update_obstacle(&mut self, n: Node, blocked: bool) {
        let current_passable = self.graph.is_passable(n);
        if current_passable != blocked {
            return;
        }
        self.graph.set_passable(n, !blocked);

        for neighbor in n.neighbors(self.graph.width, self.graph.height, self.graph.depth) {
            if self.graph.in_bounds(neighbor) {
                self.update_vertex(neighbor);
            }
        }
        self.update_vertex(n);
    }

    pub fn move_start(&mut self, new_start: Node) {
        self.km += self.graph.heuristic(self.start, new_start);
        self.start = new_start;
    }

    pub fn start(&self) -> Node {
        self.start
    }

    pub fn goal(&self) -> Node {
        self.goal
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn small_graph() -> GridGraph {
        let mut g = GridGraph::new(5, 5, 1);
        for x in 0..5 {
            for y in 0..5 {
                if x == 2 && y > 0 && y < 4 {
                    g.set_passable(Node(x, y, 0), false);
                }
            }
        }
        g
    }

    #[test]
    fn test_astar_finds_path() {
        let g = small_graph();
        let start = Node(0, 0, 0);
        let goal = Node(4, 0, 0);
        let result = astar(&g, start, goal);
        assert!(result.is_some());
        let (path, cost) = result.unwrap();
        assert_eq!(path[0], start);
        assert_eq!(path[path.len() - 1], goal);
        assert!(cost > 0.0);
    }

    #[test]
    fn test_astar_no_path() {
        let mut g = GridGraph::new(3, 3, 1);
        for x in 0..3 {
            for y in 0..3 {
                g.set_passable(Node(x, y, 0), false);
            }
        }
        let result = astar(&g, Node(0, 0, 0), Node(2, 2, 0));
        assert!(result.is_none());
    }

    #[test]
    fn test_dstar_lite_finds_path() {
        let g = small_graph();
        let start = Node(0, 0, 0);
        let goal = Node(4, 0, 0);
        let mut dl = DStarLite::new(g, start, goal);
        let path = dl.get_path();
        assert!(path.is_some());
        let p = path.unwrap();
        assert_eq!(p[0], start);
        assert_eq!(p[p.len() - 1], goal);
    }

    #[test]
    fn test_dstar_lite_replan() {
        let mut g = GridGraph::new(8, 8, 1);
        // Block a horizontal strip in the middle
        for x in 2..6 {
            g.set_passable(Node(x, 4, 0), false);
        }
        let start = Node(0, 4, 0);
        let goal = Node(7, 4, 0);
        let mut dl = DStarLite::new(g, start, goal);
        let path1 = dl.get_path();
        assert!(path1.is_some());

        // After moving to node (1,4,0), discover a new blockage
        dl.move_start(Node(1, 4, 0));
        dl.update_obstacle(Node(2, 3, 0), true);
        dl.update_obstacle(Node(2, 5, 0), true);
        let path2 = dl.get_path();
        assert!(path2.is_some());
    }
}
