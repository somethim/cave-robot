use shared::{TILE_HOLE, TILE_RAMP};

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
    println!("ramps connect {} levels", cave.size_z.saturating_sub(TILE_RAMP as usize));

    for z in 0..cave.size_z {
        let holes = cave.grid[z]
            .iter()
            .flat_map(|row| row.iter())
            .filter(|&&t| t == TILE_HOLE)
            .count();
        println!("layer {z}: {holes} holes");
    }
}
