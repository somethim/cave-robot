use shared::robot::{LidarScan, OccupancyGrid, Pose};
use shared::GAZEBO_LEVEL_SPACING_METERS;

/// A single particle in the particle filter
#[derive(Clone, Debug)]
struct Particle {
    pose: Pose,
    weight: f64,
}

/// Particle filter SLAM — Simultaneous Localization and Mapping
/// Uses a particle filter for pose estimation and an occupancy grid for mapping.
pub struct Slam {
    particles: Vec<Particle>,
    num_particles: usize,
    pub map: OccupancyGrid,
    alpha: f64,
    laser_stddev: f64,
}

impl Slam {
    pub fn new(
        width: usize,
        height: usize,
        depth: usize,
        num_particles: usize,
        initial_pose: Pose,
    ) -> Self {
        let particles: Vec<Particle> = (0..num_particles)
            .map(|_| {
                let pose = Pose::new(
                    initial_pose.x + rand::random::<f64>() * 0.5 - 0.25,
                    initial_pose.y + rand::random::<f64>() * 0.5 - 0.25,
                    initial_pose.z + rand::random::<f64>() * 0.3 - 0.15,
                    initial_pose.yaw + rand::random::<f64>() * 0.2 - 0.1,
                );
                Particle {
                    pose,
                    weight: 1.0 / num_particles as f64,
                }
            })
            .collect();

        Self {
            particles,
            num_particles,
            map: OccupancyGrid::new(width, height, depth),
            alpha: 0.1,
            laser_stddev: 0.3,
        }
    }

    /// Motion model: predict new poses from control command
    pub fn predict(&mut self, linear: f64, angular: f64, dt: f64) {
        for p in &mut self.particles {
            let noise_linear = rand::random::<f64>() * self.alpha * linear.abs().max(0.01);
            let noise_angular = rand::random::<f64>() * self.alpha * angular.abs().max(0.01);
            let noise_z = rand::random::<f64>() * self.alpha * 0.5;

            let dist = (linear + noise_linear) * dt;
            let da = (angular + noise_angular) * dt;

            p.pose.x += p.pose.yaw.cos() * dist;
            p.pose.y += p.pose.yaw.sin() * dist;
            p.pose.z += noise_z * dt;
            p.pose.yaw += da;
        }
    }

    /// Sensor model + resample + map update
    pub fn update(&mut self, scan: &LidarScan) {
        let n_h = scan.num_horizontal();
        let n_v = scan.num_vertical();

        let poses: Vec<Pose> = self.particles.iter().map(|p| p.pose).collect();
        let mut weights = Vec::with_capacity(self.num_particles);

        for pose in &poses {
            let mut likelihood = 0.0;
            let mut count = 0;

            for vi in 0..n_v {
                let elev =
                    scan.angle_min_vertical + vi as f64 * scan.angle_increment_vertical;
                for hi in 0..n_h {
                    if count >= 20 {
                        break;
                    }
                    let az = scan.angle_min + hi as f64 * scan.angle_increment;
                    let measured = scan.range(hi, vi);
                    if measured >= scan.range_max - 0.01 {
                        continue;
                    }

                    let expected = self.raycast(*pose, az, elev, scan.range_max);

                    let diff = measured - expected;
                    let prob =
                        (-diff * diff / (2.0 * self.laser_stddev * self.laser_stddev)).exp();
                    likelihood += prob;
                    count += 1;
                }
                if count >= 20 {
                    break;
                }
            }

            weights.push(likelihood.max(1e-10));
        }

        let sum: f64 = weights.iter().sum();
        if sum > 0.0 {
            for (p, w) in self.particles.iter_mut().zip(weights.iter()) {
                p.weight = w / sum;
            }
        }

        self.resample();
        self.update_map(scan);
    }

    fn raycast(&self, origin: Pose, azimuth: f64, elevation: f64, max_range: f64) -> f64 {
        let global_az = origin.yaw + azimuth;
        let dx = global_az.cos() * elevation.cos();
        let dy = global_az.sin() * elevation.cos();
        let dz = elevation.sin();

        let step = 0.2;
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
            let cz = pz as usize;
            if cx >= self.map.width || cy >= self.map.height || cz >= self.map.depth {
                return dist;
            }
            if self.map.is_occupied(cx, cy, cz) {
                return dist;
            }
        }
    }

    fn resample(&mut self) {
        let mut new_particles: Vec<Particle> = Vec::with_capacity(self.num_particles);

        let mut cumsum: Vec<f64> = Vec::with_capacity(self.num_particles);
        let mut total = 0.0;
        for p in &self.particles {
            total += p.weight;
            cumsum.push(total);
        }

        if total == 0.0 {
            return;
        }

        let step = 1.0 / self.num_particles as f64;
        let start: f64 = rand::random::<f64>() * step;

        for i in 0..self.num_particles {
            let r = start + i as f64 * step;
            let idx = cumsum
                .partition_point(|&c| c / total < r)
                .min(self.num_particles - 1);
            let mut p = self.particles[idx].clone();
            p.weight = 1.0 / self.num_particles as f64;
            new_particles.push(p);
        }

        self.particles = new_particles;
    }

    fn update_map(&mut self, scan: &LidarScan) {
        let pose = self.estimated_pose();
        let (x0, y0, z0) = pose.cell();

        let n_h = scan.num_horizontal();
        let n_v = scan.num_vertical();

        for vi in 0..n_v {
            let elev = scan.angle_min_vertical + vi as f64 * scan.angle_increment_vertical;
            for hi in 0..n_h {
                let az = scan.angle_min + hi as f64 * scan.angle_increment;
                let measured = scan.range(hi, vi);
                if measured >= scan.range_max - 0.01 {
                    continue;
                }

                let global_az = pose.yaw + az;
                let dx = global_az.cos() * elev.cos();
                let dy = global_az.sin() * elev.cos();
                let dz = elev.sin();

                let physical_z = shared::gazebo_physical_z(pose.z);
                let x1 = (pose.x + dx * measured) as usize;
                let y1 = (pose.y + dy * measured) as usize;
                let z1 = ((physical_z + dz * measured) / GAZEBO_LEVEL_SPACING_METERS) as usize;

                self.map.update_ray(x0, y0, z0, x1, y1, z1);
            }
        }
    }

    pub fn estimated_pose(&self) -> Pose {
        let mut avg_x = 0.0;
        let mut avg_y = 0.0;
        let mut avg_z = 0.0;
        let mut sin_sum = 0.0;
        let mut cos_sum = 0.0;

        for p in &self.particles {
            avg_x += p.pose.x * p.weight;
            avg_y += p.pose.y * p.weight;
            avg_z += p.pose.z * p.weight;
            sin_sum += p.pose.yaw.sin() * p.weight;
            cos_sum += p.pose.yaw.cos() * p.weight;
        }

        Pose::new(avg_x, avg_y, avg_z, sin_sum.atan2(cos_sum))
    }

    pub fn particle_poses(&self) -> Vec<Pose> {
        self.particles.iter().map(|p| p.pose).collect()
    }

    pub fn map(&self) -> &OccupancyGrid {
        &self.map
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use shared::cave::Cave;
    use shared::robot::LidarConfig;

    #[test]
    fn test_slam_initialization() {
        let pose = Pose::new(1.0, 1.0, 0.0, 0.0);
        let slam = Slam::new(10, 10, 3, 100, pose);
        assert_eq!(slam.particles.len(), 100);
        let est = slam.estimated_pose();
        assert!((est.x - 1.0).abs() < 1.0);
    }

    #[test]
    fn test_slam_predict() {
        let pose = Pose::new(1.0, 1.0, 0.0, 0.0);
        let mut slam = Slam::new(10, 10, 3, 50, pose);
        let before = slam.estimated_pose();
        slam.predict(1.0, 0.0, 0.1);
        let after = slam.estimated_pose();
        assert!(after.x > before.x);
    }

    #[test]
    fn test_slam_update_map() {
        let mut cave = Cave::new(10, 10, 3);
        for x in 5..10 {
            for y in 0..10 {
                cave.grid[0][y][x] = 1;
                cave.grid[1][y][x] = 1;
            }
        }
        cave.start = (1, 5, 0);
        cave.end = (8, 5, 0);

        let pose = Pose::new(1.5, 5.0, 0.25, 0.0);
        let config = LidarConfig::default();
        let scan = LidarScan::simulate(pose, &cave, &config);

        let mut slam = Slam::new(10, 10, 3, 100, pose);
        slam.predict(1.0, 0.0, 0.1);
        slam.update(&scan);

        let est = slam.estimated_pose();
        assert!((est.x - 1.5).abs() < 1.0);

        let hit_wall = (0..5).any(|x| slam.map.is_occupied(x + 5, 5, 0));
        assert!(hit_wall, "SLAM should mark wall cells as occupied");
    }
}
