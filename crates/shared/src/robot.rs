use crate::cave::{is_passable, Cave};

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub yaw: f64,
}

impl Pose {
    pub fn new(x: f64, y: f64, z: f64, yaw: f64) -> Self {
        Self { x, y, z, yaw }
    }

    pub fn cell(&self) -> (usize, usize, usize) {
        (
            self.x.floor() as usize,
            self.y.floor() as usize,
            self.z as usize,
        )
    }

    pub fn distance_to(&self, other: &Pose) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

#[derive(Clone, Debug)]
pub struct LidarConfig {
    pub num_rays_horizontal: usize,
    pub num_rays_vertical: usize,
    pub max_range: f64,
    pub noise_stddev: f64,
    pub angle_min: f64,
    pub angle_max: f64,
    pub angle_min_vertical: f64,
    pub angle_max_vertical: f64,
}

impl Default for LidarConfig {
    fn default() -> Self {
        Self {
            num_rays_horizontal: 180,
            num_rays_vertical: 32,
            max_range: 8.0,
            noise_stddev: 0.05,
            angle_min: -std::f64::consts::PI,
            angle_max: std::f64::consts::PI,
            angle_min_vertical: -0.6981317,
            angle_max_vertical: 0.6981317,
        }
    }
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct LidarScan {
    pub ranges: Vec<f64>,
    pub angle_min: f64,
    pub angle_max: f64,
    pub angle_increment: f64,
    #[serde(default)]
    pub angle_min_vertical: f64,
    #[serde(default)]
    pub angle_max_vertical: f64,
    #[serde(default)]
    pub angle_increment_vertical: f64,
    pub range_min: f64,
    pub range_max: f64,
    /// Pose in cave-grid coordinates at the time the scan was taken.
    /// Provided by the ROS node so the robot binary can use the actual
    /// Gazebo pose instead of dead-reckoning.
    #[serde(default)]
    pub pose: Option<Pose>,
}

impl LidarScan {
    pub fn range(&self, h: usize, v: usize) -> f64 {
        let cols = self.num_horizontal();
        self.ranges[v * cols + h]
    }

    pub fn num_horizontal(&self) -> usize {
        if self.angle_increment == 0.0 {
            return 1;
        }
        ((self.angle_max - self.angle_min) / self.angle_increment).round() as usize + 1
    }

    pub fn num_vertical(&self) -> usize {
        if self.angle_increment_vertical == 0.0 {
            return 1;
        }
        ((self.angle_max_vertical - self.angle_min_vertical) / self.angle_increment_vertical)
            .round() as usize
            + 1
    }

    pub fn len(&self) -> usize {
        self.ranges.len()
    }

    pub fn is_empty(&self) -> bool {
        self.ranges.is_empty()
    }

    pub fn simulate(pose: Pose, cave: &Cave, config: &LidarConfig) -> Self {
        let n_h = config.num_rays_horizontal;
        let n_v = config.num_rays_vertical;
        let angle_inc = if n_h > 1 {
            (config.angle_max - config.angle_min) / (n_h - 1) as f64
        } else {
            0.0
        };
        let angle_inc_v = if n_v > 1 {
            (config.angle_max_vertical - config.angle_min_vertical) / (n_v - 1) as f64
        } else {
            0.0
        };

        let mut ranges = Vec::with_capacity(n_h * n_v);

        for vi in 0..n_v {
            let elev = config.angle_min_vertical + vi as f64 * angle_inc_v;
            for hi in 0..n_h {
                let az = config.angle_min + hi as f64 * angle_inc;
                let ray_origin = Pose::new(
                    pose.x, pose.y,
                    crate::cave::gazebo_physical_z(pose.z),
                    pose.yaw,
                );
                let base = cast_ray_3d(ray_origin, az, elev, config.max_range, cave);
                let noise: f64 = rand::random::<f64>() * config.noise_stddev;
                let range = (base + noise).clamp(0.0, config.max_range);
                ranges.push(range);
            }
        }

        Self {
            ranges,
            angle_min: config.angle_min,
            angle_max: config.angle_max,
            angle_increment: angle_inc,
            angle_min_vertical: config.angle_min_vertical,
            angle_max_vertical: config.angle_max_vertical,
            angle_increment_vertical: angle_inc_v,
            range_min: 0.0,
            range_max: config.max_range,
            pose: None,
        }
    }
}

fn cast_ray_3d(
    origin: Pose,
    azimuth: f64,
    elevation: f64,
    max_range: f64,
    cave: &Cave,
) -> f64 {
    let global_az = origin.yaw + azimuth;
    let dx = global_az.cos() * elevation.cos();
    let dy = global_az.sin() * elevation.cos();
    let dz = elevation.sin();

    let step = 0.1;
    let mut dist = 0.0;

    loop {
        dist += step;
        if dist > max_range {
            return max_range;
        }
        let px = origin.x + dx * dist;
        let py = origin.y + dy * dist;
        let pz = origin.z + dz * dist;

        if px < 0.0 || py < 0.0 || pz < 0.0 {
            return dist;
        }
        let cx = px as usize;
        let cy = py as usize;
        let cz = (pz / crate::cave::GAZEBO_LEVEL_SPACING_METERS) as usize;
        if cx >= cave.size_x || cy >= cave.size_y || cz >= cave.size_z {
            return dist;
        }
        if !is_passable(cave.grid[cz][cy][cx]) {
            return dist;
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RobotMode {
    Forward,
    Return,
}

#[derive(Clone, Debug)]
pub struct RobotState {
    pub pose: Pose,
    pub mode: RobotMode,
    pub step: usize,
}

#[derive(Clone, Copy, Debug)]
pub struct NavCommand {
    pub linear_x: f64,
    pub linear_y: f64,
    pub linear_z: f64,
    pub angular_z: f64,
}

impl NavCommand {
    pub fn forward(speed: f64) -> Self {
        Self {
            linear_x: speed,
            linear_y: 0.0,
            linear_z: 0.0,
            angular_z: 0.0,
        }
    }

    pub fn stop() -> Self {
        Self {
            linear_x: 0.0,
            linear_y: 0.0,
            linear_z: 0.0,
            angular_z: 0.0,
        }
    }

    pub fn rotate(angular_z: f64) -> Self {
        Self {
            linear_x: 0.0,
            linear_y: 0.0,
            linear_z: 0.0,
            angular_z,
        }
    }
}

pub struct OccupancyGrid {
    pub width: usize,
    pub height: usize,
    pub depth: usize,
    log_odds: Vec<f64>,
    pub resolution: f64,
}

const LOG_ODDS_OCCUPIED: f64 = 0.8;
const LOG_ODDS_FREE: f64 = -0.4;
const OCCUPIED_THRESHOLD: f64 = 0.5;

impl OccupancyGrid {
    pub fn new(width: usize, height: usize, depth: usize) -> Self {
        Self {
            width,
            height,
            depth,
            log_odds: vec![0.0; width * height * depth],
            resolution: 1.0,
        }
    }

    pub fn index(&self, x: usize, y: usize, z: usize) -> usize {
        z * self.width * self.height + y * self.width + x
    }

    pub fn probability(&self, x: usize, y: usize, z: usize) -> f64 {
        let lo = self.log_odds[self.index(x, y, z)];
        1.0 - 1.0 / (1.0 + lo.exp())
    }

    pub fn update_cell(&mut self, x: usize, y: usize, z: usize, occupied: bool) {
        if x >= self.width || y >= self.height || z >= self.depth {
            return;
        }
        let idx = self.index(x, y, z);
        if occupied {
            self.log_odds[idx] += LOG_ODDS_OCCUPIED;
        } else {
            self.log_odds[idx] += LOG_ODDS_FREE;
        }
    }

    pub fn update_ray(&mut self, x0: usize, y0: usize, z0: usize, x1: usize, y1: usize, z1: usize) {
        let cells = bresenham_3d(x0, y0, z0, x1, y1, z1);
        if cells.is_empty() {
            return;
        }
        self.update_cell(x0, y0, z0, false);
        if cells.len() > 2 {
            for &(x, y, z) in &cells[1..cells.len() - 1] {
                self.update_cell(x, y, z, false);
            }
        }
        let last = cells[cells.len() - 1];
        if (last.0, last.1, last.2) != (x0, y0, z0) {
            self.update_cell(last.0, last.1, last.2, true);
        }
    }

    pub fn is_occupied(&self, x: usize, y: usize, z: usize) -> bool {
        if x >= self.width || y >= self.height || z >= self.depth {
            return true;
        }
        self.probability(x, y, z) > OCCUPIED_THRESHOLD
    }

    pub fn to_passable_grid(&self) -> Vec<Vec<Vec<bool>>> {
        let mut grid = vec![vec![vec![false; self.width]; self.height]; self.depth];
        for (z, layer) in grid.iter_mut().enumerate() {
            for (y, row) in layer.iter_mut().enumerate() {
                for (x, cell) in row.iter_mut().enumerate() {
                    *cell = !self.is_occupied(x, y, z);
                }
            }
        }
        grid
    }
}

fn bresenham_3d(
    x0: usize,
    y0: usize,
    z0: usize,
    x1: usize,
    y1: usize,
    z1: usize,
) -> Vec<(usize, usize, usize)> {
    let target = [x1 as isize, y1 as isize, z1 as isize];
    let origin = [x0 as isize, y0 as isize, z0 as isize];
    let d = [
        (target[0] - origin[0]).abs(),
        (target[1] - origin[1]).abs(),
        (target[2] - origin[2]).abs(),
    ];
    let s = [
        if target[0] >= origin[0] { 1 } else { -1 },
        if target[1] >= origin[1] { 1 } else { -1 },
        if target[2] >= origin[2] { 1 } else { -1 },
    ];

    let major = if d[0] >= d[1] && d[0] >= d[2] {
        0
    } else if d[1] >= d[0] && d[1] >= d[2] {
        1
    } else {
        2
    };

    let a = (major + 1) % 3;
    let b = (major + 2) % 3;

    let mut c = origin;
    let mut p1 = 2 * d[a] - d[major];
    let mut p2 = 2 * d[b] - d[major];

    let mut cells = Vec::new();
    loop {
        cells.push((c[0] as usize, c[1] as usize, c[2] as usize));
        if c[major] == target[major] {
            break;
        }
        c[major] += s[major];
        if p1 >= 0 {
            c[a] += s[a];
            p1 -= 2 * d[major];
        }
        if p2 >= 0 {
            c[b] += s[b];
            p2 -= 2 * d[major];
        }
        p1 += 2 * d[a];
        p2 += 2 * d[b];
    }

    cells
}
