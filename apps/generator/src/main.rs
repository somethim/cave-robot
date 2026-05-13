mod map;

fn main() {
    let size_x = 64;
    let size_y = 32;
    let size_z = 4;
    let seed = 42;

    let mut m = map::Map::new(size_x, size_y, size_z);
    m.generate(seed);
    m.cave.display();
}
