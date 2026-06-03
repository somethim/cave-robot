pub mod cellular;
pub mod connect;
pub mod dead_ends;
pub mod flood_fill;
pub mod holes;
pub mod init;
pub mod ramps;
pub mod start_end;

use rand::SeedableRng;
use shared::{Cave, TILE_FLOOR};

#[derive(Clone, Copy)]
pub struct GeneratorConfig {
    pub fill_confidence: u32,
    pub wall_threshold: u32,
    pub floor_threshold: u32,
    pub smooth_iterations: usize,
    pub dead_end_percent: usize,
    pub hole_percent: usize,
}

impl GeneratorConfig {
    pub fn from_env() -> Self {
        Self {
            fill_confidence: shared::env_or("CAVE_FILL_CONFIDENCE", 50),
            wall_threshold: shared::env_or("CAVE_WALL_THRESHOLD", 6),
            floor_threshold: shared::env_or("CAVE_FLOOR_THRESHOLD", 3),
            smooth_iterations: shared::env_or("CAVE_SMOOTH_ITERATIONS", 6),
            dead_end_percent: shared::env_or("CAVE_DEAD_END_PERCENT", 1),
            hole_percent: shared::env_or("CAVE_HOLE_PERCENT", 1),
        }
    }
}

impl Default for GeneratorConfig {
    fn default() -> Self {
        Self::from_env()
    }
}

pub struct Map {
    pub cave: Cave,
    pub(crate) grid2: Vec<Vec<Vec<u8>>>,
    pub(crate) config: GeneratorConfig,
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
            grid2: vec![vec![vec![TILE_FLOOR; size_x]; size_y]; size_z],
            config,
        }
    }

    pub fn generate(&mut self, seed: u64) {
        let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
        for level in 0..self.cave.size_z {
            self.initmap(level, &mut rng);
            for _ in 0..self.config.smooth_iterations {
                self.step(level);
            }
            self.connect_regions(level);
            let floor_count = self.cave.grid[level]
                .iter()
                .flat_map(|row| row.iter())
                .filter(|&&t| t == TILE_FLOOR)
                .count();
            let dead_end_count = (floor_count * self.config.dead_end_percent) / 100;
            self.carve_dead_ends(level, &mut rng, dead_end_count);
        }
        self.place_ramps(&mut rng);
        let total_floor: usize = self.cave.grid
            .iter()
            .flat_map(|level| level.iter())
            .flat_map(|row| row.iter())
            .filter(|&&t| t == TILE_FLOOR)
            .count();
        let hole_count = (total_floor * self.config.hole_percent) / 100;
        self.place_holes(&mut rng, hole_count);
        self.connect_regions_3d();
        self.pick_start_end(&mut rng);
    }
}
