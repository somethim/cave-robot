mod map;

fn main() {
    let size_x = 64;
    let size_y = 32;
    let size_z = 4;
    let seed = 42;

    let config = map::GeneratorConfig {
        fill_confidence: 50,
        wall_threshold: 6,
        floor_threshold: 3,
        smooth_iterations: 6,
        dead_end_count: 10,
    };

    let mut m = map::Map::with_config(size_x, size_y, size_z, config);
    m.generate(seed);
    m.cave.display();
}
