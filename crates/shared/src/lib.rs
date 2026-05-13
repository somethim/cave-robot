use std::str::FromStr;

pub fn load_env() {
    dotenvy::dotenv().ok();
}

pub fn env_or<T: FromStr>(key: &str, default: T) -> T {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

pub const TILE_FLOOR: u8 = 0;
pub const TILE_WALL: u8 = 1;
pub const TILE_RAMP: u8 = 2;
pub const TILE_HOLE: u8 = 3;

pub fn is_passable(tile: u8) -> bool {
    tile == TILE_FLOOR || tile == TILE_RAMP || tile == TILE_HOLE
}

pub fn allows_vertical(tile: u8) -> bool {
    tile == TILE_RAMP || tile == TILE_HOLE
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Cave {
    pub grid: Vec<Vec<Vec<u8>>>,
    pub size_x: usize,
    pub size_y: usize,
    pub size_z: usize,
    pub start: (usize, usize, usize),
    pub end: (usize, usize, usize),
}

impl Cave {
    pub fn new(size_x: usize, size_y: usize, size_z: usize) -> Self {
        Self {
            grid: vec![vec![vec![TILE_FLOOR; size_x]; size_y]; size_z],
            size_x,
            size_y,
            size_z,
            start: (0, 0, 0),
            end: (0, 0, 0),
        }
    }

    pub fn display(&self) {
        for level in 0..self.size_z {
            println!();
            println!("--- Slice {} (z={}) ---", level, level);
            for row in 0..self.size_y {
                for col in 0..self.size_x {
                    if (col, row, level) == self.start {
                        print!("S");
                    } else if (col, row, level) == self.end {
                        print!("E");
                    } else {
                        match self.grid[level][row][col] {
                            TILE_WALL => print!("#"),
                            TILE_RAMP => print!("%"),
                            TILE_HOLE => print!("O"),
                            _ => print!("."),
                        }
                    }
                }
                println!();
            }
        }
        println!();
    }
}
