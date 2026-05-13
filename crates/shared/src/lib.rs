use std::str::FromStr;

pub fn load_env() {
    dotenvy::dotenv().ok();
}

pub fn env_or<T: FromStr>(key: &str, default: T) -> T {
    std::env::var(key).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

pub const TILE_FLOOR: u8 = 0;
pub const TILE_WALL: u8 = 1;

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Cave {
    pub grid: Vec<Vec<Vec<u8>>>,
    pub size_x: usize,
    pub size_y: usize,
    pub size_z: usize,
    pub start: (usize, usize, usize),
    pub end: (usize, usize, usize),
    pub stairs: Vec<(usize, usize)>,
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
            stairs: Vec::new(),
        }
    }

    pub fn display(&self) {
        for level in 0..self.size_z {
            println!();
            println!("--- Floor {} ---", level);
            for row in 0..self.size_y {
                for col in 0..self.size_x {
                    if (col, row, level) == self.start {
                        print!("S");
                    } else if (col, row, level) == self.end {
                        print!("E");
                    } else if level > 0 && self.stairs.get(level - 1) == Some(&(col, row)) {
                        print!("U");
                    } else if self.stairs.get(level) == Some(&(col, row)) {
                        print!("D");
                    } else {
                        match self.grid[level][row][col] {
                            TILE_WALL => print!("#"),
                            _ => print!("."),
                        }
                    }
                }
                println!();
            }
            if let Some(&(stair_col, stair_row)) = self.stairs.get(level) {
                println!("  ↓ stairs down at ({}, {})", stair_col, stair_row);
            }
        }
        println!();
    }
}
