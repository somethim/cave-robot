pub const TILE_FLOOR: u8 = 0;
pub const TILE_WALL: u8 = 1;
pub const TILE_RAMP: u8 = 2;
pub const TILE_HOLE: u8 = 3;

// Gazebo cave generation conventions are fixed here so the generator, launch
// code, and spawn logic all agree on scale and coordinate mapping.
pub const GAZEBO_VOXEL_SIZE_METERS: f64 = 1.0;
pub const GAZEBO_LEVEL_SPACING_METERS: f64 = 3.0;
pub const GAZEBO_CELL_CENTER_Z_BIAS_METERS: f64 = 0.5;
pub const GAZEBO_ROBOT_SPAWN_Z_OFFSET_METERS: f64 = 0.25;

pub fn gazebo_level_base_z(level: usize) -> f64 {
    level as f64 * GAZEBO_LEVEL_SPACING_METERS
}

pub fn gazebo_cell_center(cell: (usize, usize, usize)) -> (f64, f64, f64) {
    let (x, y, z) = cell;
    (
        (x as f64 + 0.5) * GAZEBO_VOXEL_SIZE_METERS,
        (y as f64 + 0.5) * GAZEBO_VOXEL_SIZE_METERS,
        gazebo_level_base_z(z) + GAZEBO_CELL_CENTER_Z_BIAS_METERS * GAZEBO_VOXEL_SIZE_METERS,
    )
}

pub fn gazebo_robot_spawn_position(cell: (usize, usize, usize)) -> (f64, f64, f64) {
    let (x, y, z) = gazebo_cell_center(cell);
    (x, y, z + GAZEBO_ROBOT_SPAWN_Z_OFFSET_METERS)
}

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
