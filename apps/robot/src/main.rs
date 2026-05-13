fn main() {
    shared::load_env();

    let path = shared::env_or("CAVE_FILE", "cave.json".to_string());
    let json = std::fs::read_to_string(&path).unwrap();
    let cave: shared::Cave = serde_json::from_str(&json).unwrap();
    println!(
        "loaded cave: {}×{}×{}",
        cave.size_x, cave.size_y, cave.size_z
    );
    println!("start: {:?}, end: {:?}", cave.start, cave.end);
    println!("stairs: {:?}", cave.stairs);
}
