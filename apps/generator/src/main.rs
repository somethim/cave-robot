fn main() {
    shared::load_env();

    let size_x = shared::env_or("CAVE_SIZE_X", 64usize);
    let size_y = shared::env_or("CAVE_SIZE_Y", 32usize);
    let size_z = shared::env_or("CAVE_SIZE_Z", 4usize);
    let seed = shared::env_or("CAVE_SEED", 42u64);

    let config = generator::map::GeneratorConfig::from_env();
    let mut m = generator::map::Map::with_config(size_x, size_y, size_z, config);
    m.generate(seed);

    let path_str = shared::env_or("CAVE_FILE", "cave.json".to_string());
    let path = std::path::Path::new(&path_str);
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).unwrap();
    }
    let json = serde_json::to_string_pretty(&m.cave).unwrap();
    std::fs::write(path, json).unwrap();
    eprintln!("cave written to {}", path.display());

    m.cave.display();
}
