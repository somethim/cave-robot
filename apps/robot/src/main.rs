use generator::map::{GeneratorConfig, Map};
use std::path::PathBuf;

fn main() {
    let size_x = 64;
    let size_y = 32;
    let size_z = 4;
    let seed = 42;

    let config = GeneratorConfig {
        fill_confidence: 50,
        wall_threshold: 6,
        floor_threshold: 3,
        smooth_iterations: 6,
        dead_end_count: 10,
    };

    let mut map = Map::with_config(size_x, size_y, size_z, config);
    map.generate(seed);
    map.cave.display();

    let path = PathBuf::from(
        std::env::var("CAVE_FILE").unwrap_or_else(|_| "/tmp/cave.json".into()),
    );
    let json = serde_json::to_string_pretty(&map.cave).unwrap();
    std::fs::write(&path, json).unwrap();
    eprintln!("cave written to {}", path.display());
}
