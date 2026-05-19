use generator::world;
use shared::Cave;
use std::path::PathBuf;

fn main() {
    shared::load_env();

    let cave_path = shared::env_or("CAVE_FILE", "cave.json".to_string());
    let world_path = PathBuf::from(world::default_world_file());

    let cave_json = std::fs::read_to_string(&cave_path).unwrap_or_else(|error| {
        panic!("failed to read cave file '{}': {error}", cave_path);
    });
    let cave: Cave = serde_json::from_str(&cave_json).unwrap_or_else(|error| {
        panic!("failed to parse cave file '{}': {error}", cave_path);
    });

    let metadata = world::write_world_from_cave(&cave, &world_path).unwrap_or_else(|error| {
        panic!("failed to write world file '{}': {error}", world_path.display());
    });

    eprintln!("world written to {}", metadata.world_file);
    eprintln!("metadata written to {}", world::metadata_file_for(&world_path).display());
    eprintln!("merged wall boxes: {}", metadata.merged_box_count);
    eprintln!(
        "robot spawn: ({:.3}, {:.3}, {:.3})",
        metadata.robot_spawn_position.0,
        metadata.robot_spawn_position.1,
        metadata.robot_spawn_position.2,
    );
}
