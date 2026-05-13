pub mod cellular;
pub mod connect;
pub mod dead_ends;
pub mod flood_fill;
pub mod init;
pub mod stairs;
pub mod start_end;

use rand::SeedableRng;
use shared::{Cave, TILE_FLOOR};

#[derive(Clone, Copy)]
pub struct GeneratorConfig {
    pub fill_confidence: u32,
    pub wall_threshold: u32,
    pub floor_threshold: u32,
    pub smooth_iterations: usize,
    pub dead_end_count: usize,
}

impl GeneratorConfig {
    pub fn from_env() -> Self {
        Self {
            fill_confidence: shared::env_or("CAVE_FILL_CONFIDENCE", 50),
            wall_threshold: shared::env_or("CAVE_WALL_THRESHOLD", 6),
            floor_threshold: shared::env_or("CAVE_FLOOR_THRESHOLD", 3),
            smooth_iterations: shared::env_or("CAVE_SMOOTH_ITERATIONS", 6),
            dead_end_count: shared::env_or("CAVE_DEAD_END_COUNT", 8),
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
    pub(crate) grid2: Vec<Vec<u8>>,
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
            grid2: vec![vec![TILE_FLOOR; size_x]; size_y],
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
            self.carve_dead_ends(level, &mut rng, self.config.dead_end_count);
        }
        self.place_stairs(&mut rng);
        for level in 0..self.cave.size_z {
            self.connect_regions(level);
        }
        self.pick_start_end(&mut rng);
    }
}
