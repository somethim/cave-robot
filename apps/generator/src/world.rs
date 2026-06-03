use serde::Serialize;
use shared::{
    Cave, GAZEBO_LEVEL_SPACING_METERS, GAZEBO_VOXEL_SIZE_METERS, TILE_HOLE, TILE_RAMP,
    TILE_WALL, allows_vertical,
    gazebo_level_base_z, gazebo_physical_z, is_passable,
};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

#[derive(Serialize)]
pub struct WorldMetadata {
    pub voxel_size_meters: f64,
    pub world_file: String,
    pub start_cell: (usize, usize, usize),
    pub end_cell: (usize, usize, usize),
    pub robot_spawn_position: (f64, f64, f64),
    pub merged_box_count: usize,
}

#[derive(Clone, Copy)]
struct WallBox {
    x: usize,
    y: usize,
    z: usize,
    width: usize,
    depth: usize,
}

#[derive(Clone, Copy)]
struct SlabBox {
    x: usize,
    y: usize,
    z: usize,
    width: usize,
    depth: usize,
}

impl SlabBox {
    fn size(self, thickness: f64) -> (f64, f64, f64) {
        (
            self.width as f64 * GAZEBO_VOXEL_SIZE_METERS,
            self.depth as f64 * GAZEBO_VOXEL_SIZE_METERS,
            thickness,
        )
    }

    fn floor_pose(self, thickness: f64) -> (f64, f64, f64) {
        (
            (self.x as f64 + self.width as f64 / 2.0) * GAZEBO_VOXEL_SIZE_METERS,
            (self.y as f64 + self.depth as f64 / 2.0) * GAZEBO_VOXEL_SIZE_METERS,
            gazebo_level_base_z(self.z) + thickness / 2.0,
        )
    }

    fn ceiling_pose(self, thickness: f64) -> (f64, f64, f64) {
        (
            (self.x as f64 + self.width as f64 / 2.0) * GAZEBO_VOXEL_SIZE_METERS,
            (self.y as f64 + self.depth as f64 / 2.0) * GAZEBO_VOXEL_SIZE_METERS,
            gazebo_level_base_z(self.z) + GAZEBO_LEVEL_SPACING_METERS - thickness / 2.0,
        )
    }
}

const SLAB_THICKNESS_METERS: f64 = 0.1;
const RAMP_SURFACE_THICKNESS_METERS: f64 = 0.08;
const RAMP_SURFACE_WIDTH_METERS: f64 = 0.7;
const RAMP_HORIZONTAL_RUN_METERS: f64 = 2.0; // 45° slope with 2m level spacing
const HOLE_SHAFT_WALL_THICKNESS_METERS: f64 = 0.08;

const WALL_COLOR: &str = "0.55 0.55 0.55 1";
const FLOOR_COLOR: &str = "0.82 0.82 0.82 1";
const RAMP_COLOR: &str = "0.20 0.80 0.25 1";
const HOLE_COLOR: &str = "0.88 0.20 0.20 1";

impl WallBox {
    fn pose(self) -> (f64, f64, f64) {
        (
            (self.x as f64 + self.width as f64 / 2.0) * GAZEBO_VOXEL_SIZE_METERS,
            (self.y as f64 + self.depth as f64 / 2.0) * GAZEBO_VOXEL_SIZE_METERS,
            // Centre of the full level height so walls span floor-to-ceiling
            gazebo_level_base_z(self.z) + GAZEBO_LEVEL_SPACING_METERS / 2.0,
        )
    }

    fn size(self) -> (f64, f64, f64) {
        (
            self.width as f64 * GAZEBO_VOXEL_SIZE_METERS,
            self.depth as f64 * GAZEBO_VOXEL_SIZE_METERS,
            GAZEBO_LEVEL_SPACING_METERS, // full floor-to-ceiling height
        )
    }
}

pub fn default_world_file() -> String {
    shared::env_or(
        "CAVE_WORLD_FILE",
        "./ros/src/cave_robot_gazebo/worlds/generated/cave.world".to_string(),
    )
}

pub fn default_robot_sdf_file() -> String {
    shared::env_or(
        "CAVE_ROBOT_SDF_FILE",
        "./ros/src/cave_robot_description/urdf/cave_robot.sdf".to_string(),
    )
}

pub fn metadata_file_for(world_path: &Path) -> PathBuf {
    let stem = world_path
        .file_stem()
        .and_then(|stem| stem.to_str())
        .unwrap_or("cave");
    world_path.with_file_name(format!("{stem}.meta.json"))
}

pub fn write_world_from_cave(cave: &Cave, world_path: &Path) -> std::io::Result<WorldMetadata> {
    let wall_boxes = merge_wall_boxes(cave);
    let slab_boxes = merge_slab_boxes(cave);
    let world = render_world(cave, &wall_boxes, &slab_boxes);
    if let Some(parent) = world_path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(world_path, world)?;

    let metadata = WorldMetadata {
        voxel_size_meters: GAZEBO_VOXEL_SIZE_METERS,
        world_file: world_path.display().to_string(),
        start_cell: cave.start,
        end_cell: cave.end,
        robot_spawn_position: (
            cave.start.0 as f64 + 0.5,
            cave.start.1 as f64 + 0.5,
            gazebo_physical_z(cave.start.2 as f64),
        ),
        merged_box_count: wall_boxes.len(),
    };
    let metadata_path = metadata_file_for(world_path);
    std::fs::write(metadata_path, serde_json::to_string_pretty(&metadata)?)?;
    Ok(metadata)
}

fn merge_wall_boxes(cave: &Cave) -> Vec<WallBox> {
    merge_rectangles(cave, |tile| tile == TILE_WALL)
        .into_iter()
        .map(|slab| WallBox {
            x: slab.x,
            y: slab.y,
            z: slab.z,
            width: slab.width,
            depth: slab.depth,
        })
        .collect()
}

fn merge_slab_boxes(cave: &Cave) -> Vec<SlabBox> {
    merge_rectangles(cave, |tile| is_passable(tile) && !allows_vertical(tile))
}

fn merge_rectangles<F>(cave: &Cave, include: F) -> Vec<SlabBox>
where
    F: Fn(u8) -> bool,
{
    let mut boxes = Vec::new();

    for z in 0..cave.size_z {
        let mut visited = vec![vec![false; cave.size_x]; cave.size_y];
        for y in 0..cave.size_y {
            for x in 0..cave.size_x {
                if visited[y][x] || !include(cave.grid[z][y][x]) {
                    continue;
                }

                let mut width = 1;
                while x + width < cave.size_x
                    && !visited[y][x + width]
                    && include(cave.grid[z][y][x + width])
                {
                    width += 1;
                }

                let mut depth = 1;
                'grow_depth: while y + depth < cave.size_y {
                    for dx in 0..width {
                        if visited[y + depth][x + dx]
                            || !include(cave.grid[z][y + depth][x + dx])
                        {
                            break 'grow_depth;
                        }
                    }
                    depth += 1;
                }

                for dy in 0..depth {
                    for dx in 0..width {
                        visited[y + dy][x + dx] = true;
                    }
                }

                boxes.push(SlabBox {
                    x,
                    y,
                    z,
                    width,
                    depth,
                });
            }
        }
    }

    boxes
}

fn render_world(cave: &Cave, wall_boxes: &[WallBox], slab_boxes: &[SlabBox]) -> String {
    let mut output = String::new();
    let start_x = cave.start.0 as f64 + 0.5;
    let start_y = cave.start.1 as f64 + 0.5;
    let start_z = gazebo_physical_z(cave.start.2 as f64);
    let end_x = cave.end.0 as f64 + 0.5;
    let end_y = cave.end.1 as f64 + 0.5;
    let end_z = gazebo_physical_z(cave.end.2 as f64);
    let robot_x = start_x;
    let robot_y = start_y;
    let robot_z = start_z;
    let robot_uri = robot_model_uri();

    output.push_str("<?xml version=\"1.0\"?>\n");
    output.push_str("<sdf version=\"1.9\">\n");
    output.push_str("  <world name=\"cave_world\">\n");
    output.push_str("    <gravity>0 0 0</gravity>\n");
    output.push_str("    <physics name=\"fast\" type=\"dart\">\n");
    output.push_str("      <max_step_size>0.01</max_step_size>\n");
    output.push_str("      <real_time_factor>1.0</real_time_factor>\n");
    output.push_str("    </physics>\n");
    output.push_str("    <scene>\n");
    output.push_str("      <ambient>0.12 0.12 0.14 1</ambient>\n");
    output.push_str("      <background>0.04 0.04 0.06 1</background>\n");
    output.push_str("    </scene>\n");

    // Dim directional fill light so in-cave point lights are the primary source
    output.push_str("    <light name=\"fill_light\" type=\"directional\">\n");
    output.push_str("      <cast_shadows>false</cast_shadows>\n");
    output.push_str("      <pose>0 0 50 0 0 0</pose>\n");
    output.push_str("      <diffuse>0.20 0.20 0.22 1</diffuse>\n");
    output.push_str("      <specular>0.05 0.05 0.05 1</specular>\n");
    output.push_str("      <attenuation><range>1000</range><constant>1</constant></attenuation>\n");
    output.push_str("      <direction>0 0 -1</direction>\n");
    output.push_str("    </light>\n");

    // Distributed in-cave point lights — one per 8×8 cell grid sector per level,
    // placed near the ceiling of the first passable cell found in each sector.
    const LIGHT_GRID: usize = 8;
    let mut light_idx = 0usize;
    for z in 0..cave.size_z {
        let lz = gazebo_level_base_z(z) + GAZEBO_LEVEL_SPACING_METERS - 0.25;
        for gy in (0..cave.size_y).step_by(LIGHT_GRID) {
            for gx in (0..cave.size_x).step_by(LIGHT_GRID) {
                // Find the first passable cell in this grid sector
                'sector: for dy in 0..LIGHT_GRID {
                    for dx in 0..LIGHT_GRID {
                        let cy = gy + dy;
                        let cx = gx + dx;
                        if cy < cave.size_y && cx < cave.size_x && is_passable(cave.grid[z][cy][cx]) {
                            let lx = (cx as f64 + 0.5) * GAZEBO_VOXEL_SIZE_METERS;
                            let ly = (cy as f64 + 0.5) * GAZEBO_VOXEL_SIZE_METERS;
                            let _ = writeln!(output, "    <light name=\"cave_light_{light_idx}\" type=\"point\">");
                            output.push_str("      <cast_shadows>false</cast_shadows>\n");
                            let _ = writeln!(output, "      <pose>{lx:.3} {ly:.3} {lz:.3} 0 0 0</pose>");
                            output.push_str("      <diffuse>1.0 0.92 0.78 1</diffuse>\n");
                            output.push_str("      <specular>0.15 0.15 0.12 1</specular>\n");
                            output.push_str("      <attenuation>\n");
                            output.push_str("        <range>14</range>\n");
                            output.push_str("        <constant>0.2</constant>\n");
                            output.push_str("        <linear>0.08</linear>\n");
                            output.push_str("        <quadratic>0.015</quadratic>\n");
                            output.push_str("      </attenuation>\n");
                            output.push_str("    </light>\n");
                            light_idx += 1;
                            break 'sector;
                        }
                    }
                }
            }
        }
    }
    output.push_str("    <model name=\"cave_walls\">\n");
    output.push_str("      <static>true</static>\n");
    output.push_str("      <link name=\"cave\">\n");

    for (index, wall_box) in wall_boxes.iter().copied().enumerate() {
        let (px, py, pz) = wall_box.pose();
        let (sx, sy, sz) = wall_box.size();
        append_box_geometry(
            &mut output,
            &format!("wall_{index}"),
            (px, py, pz),
            (sx, sy, sz),
            WALL_COLOR,
        );
    }

    for (index, slab_box) in slab_boxes.iter().copied().enumerate() {
        let slab_size = slab_box.size(SLAB_THICKNESS_METERS);

        // Floor slab at the bottom of this level
        let floor_pose = slab_box.floor_pose(SLAB_THICKNESS_METERS);
        append_box_geometry(&mut output, &format!("floor_{index}"), floor_pose, slab_size, FLOOR_COLOR);

        // Ceiling slab at the top of this level to seal each corridor (skip the roof of the top level)
        if slab_box.z < cave.size_z - 1 {
            let ceil_pose = slab_box.ceiling_pose(SLAB_THICKNESS_METERS);
            append_box_geometry(&mut output, &format!("ceiling_{index}"), ceil_pose, slab_size, FLOOR_COLOR);
        }
    }

    append_vertical_connectors(&mut output, cave);

    output.push_str("      </link>\n");
    output.push_str("    </model>\n");
    let _ = writeln!(output, "    <include>");
    let _ = writeln!(output, "      <uri>{robot_uri}</uri>");
    output.push_str("      <name>cave_robot</name>\n");
    let _ = writeln!(output, "      <pose>{robot_x:.3} {robot_y:.3} {robot_z:.3} 0 0 0</pose>");
    output.push_str("    </include>\n");
    append_marker(&mut output, "start_marker", start_x, start_y, start_z, "0.15 0.35 0.95 1");
    append_marker(&mut output, "end_marker", end_x, end_y, end_z, "0.60 0.20 0.85 1");
    output.push_str("  </world>\n");
    output.push_str("</sdf>\n");

    output
}

fn robot_model_uri() -> String {
    let path = PathBuf::from(default_robot_sdf_file());
    let canonical = path.canonicalize().unwrap_or(path);
    format!("file://{}", canonical.display())
}

fn append_box_geometry(
    output: &mut String,
    name: &str,
    pose: (f64, f64, f64),
    size: (f64, f64, f64),
    color: &str,
) {
    let (px, py, pz) = pose;
    let (sx, sy, sz) = size;
    let _ = writeln!(output, "        <collision name=\"{name}_collision\">");
    let _ = writeln!(output, "          <pose>{px:.3} {py:.3} {pz:.3} 0 0 0</pose>");
    let _ = writeln!(output, "          <geometry><box><size>{sx:.3} {sy:.3} {sz:.3}</size></box></geometry>");
    let _ = writeln!(output, "        </collision>");
    let _ = writeln!(output, "        <visual name=\"{name}_visual\">");
    let _ = writeln!(output, "          <pose>{px:.3} {py:.3} {pz:.3} 0 0 0</pose>");
    let _ = writeln!(output, "          <geometry><box><size>{sx:.3} {sy:.3} {sz:.3}</size></box></geometry>");
    output.push_str("          <material>\n");
    let _ = writeln!(output, "            <ambient>{color}</ambient>");
    let _ = writeln!(output, "            <diffuse>{color}</diffuse>");
    output.push_str("          </material>\n");
    let _ = writeln!(output, "        </visual>");
}

fn append_rotated_box_geometry(
    output: &mut String,
    name: &str,
    pose: (f64, f64, f64),
    rpy: (f64, f64, f64),
    size: (f64, f64, f64),
    color: &str,
) {
    let (px, py, pz) = pose;
    let (rr, rp, ry) = rpy;
    let (sx, sy, sz) = size;
    let _ = writeln!(output, "        <collision name=\"{name}_collision\">");
    let _ = writeln!(output, "          <pose>{px:.3} {py:.3} {pz:.3} {rr:.3} {rp:.3} {ry:.3}</pose>");
    let _ = writeln!(output, "          <geometry><box><size>{sx:.3} {sy:.3} {sz:.3}</size></box></geometry>");
    let _ = writeln!(output, "        </collision>");
    let _ = writeln!(output, "        <visual name=\"{name}_visual\">");
    let _ = writeln!(output, "          <pose>{px:.3} {py:.3} {pz:.3} {rr:.3} {rp:.3} {ry:.3}</pose>");
    let _ = writeln!(output, "          <geometry><box><size>{sx:.3} {sy:.3} {sz:.3}</size></box></geometry>");
    output.push_str("          <material>\n");
    let _ = writeln!(output, "            <ambient>{color}</ambient>");
    let _ = writeln!(output, "            <diffuse>{color}</diffuse>");
    output.push_str("          </material>\n");
    let _ = writeln!(output, "        </visual>");
}

fn append_vertical_connectors(output: &mut String, cave: &Cave) {
    let mut ramp_index = 0usize;
    let mut hole_index = 0usize;

    for z in 0..cave.size_z.saturating_sub(1) {
        for y in 0..cave.size_y {
            for x in 0..cave.size_x {
                let tile = cave.grid[z][y][x];
                let above = cave.grid[z + 1][y][x];

                if tile == TILE_RAMP && above == TILE_RAMP {
                    append_ramp_geometry(output, x, y, z, ramp_index);
                    ramp_index += 1;
                }

                if tile == TILE_HOLE && above == TILE_HOLE {
                    append_hole_geometry(output, x, y, z, hole_index);
                    hole_index += 1;
                }
            }
        }
    }
}

fn append_ramp_geometry(output: &mut String, x: usize, y: usize, z: usize, index: usize) {
    let base_x = x as f64 * GAZEBO_VOXEL_SIZE_METERS;
    let base_y = y as f64 * GAZEBO_VOXEL_SIZE_METERS;
    let lower_z = gazebo_level_base_z(z) + SLAB_THICKNESS_METERS;
    let center_z = lower_z + GAZEBO_LEVEL_SPACING_METERS / 2.0;
    let ramp_length = (RAMP_HORIZONTAL_RUN_METERS.powi(2) + GAZEBO_LEVEL_SPACING_METERS.powi(2)).sqrt();
    let pitch = -f64::atan2(GAZEBO_LEVEL_SPACING_METERS, RAMP_HORIZONTAL_RUN_METERS);
    let orientation = (x + y + z) % 4;

    let (center_x, center_y, yaw) = match orientation {
        0 => (base_x + 0.5, base_y + 0.5, 0.0),
        1 => (base_x + 0.5, base_y + 0.5, std::f64::consts::FRAC_PI_2),
        2 => (base_x + 0.5, base_y + 0.5, std::f64::consts::PI),
        _ => (base_x + 0.5, base_y + 0.5, -std::f64::consts::FRAC_PI_2),
    };

    append_rotated_box_geometry(
        output,
        &format!("ramp_{index}"),
        (center_x, center_y, center_z),
        (0.0, pitch, yaw),
        (
            ramp_length,
            RAMP_SURFACE_WIDTH_METERS,
            RAMP_SURFACE_THICKNESS_METERS,
        ),
        RAMP_COLOR,
    );
}

fn append_hole_geometry(output: &mut String, x: usize, y: usize, z: usize, index: usize) {
    let center_x = (x as f64 + 0.5) * GAZEBO_VOXEL_SIZE_METERS;
    let center_y = (y as f64 + 0.5) * GAZEBO_VOXEL_SIZE_METERS;
    let shaft_height = GAZEBO_LEVEL_SPACING_METERS;
    let center_z = gazebo_level_base_z(z) + shaft_height / 2.0;
    let inner_span = 0.55;
    let outer_offset = (inner_span + HOLE_SHAFT_WALL_THICKNESS_METERS) / 2.0;

    append_box_geometry(
        output,
        &format!("hole_{index}_north"),
        (center_x, center_y + outer_offset, center_z),
        (inner_span + HOLE_SHAFT_WALL_THICKNESS_METERS * 2.0, HOLE_SHAFT_WALL_THICKNESS_METERS, shaft_height),
        HOLE_COLOR,
    );
    append_box_geometry(
        output,
        &format!("hole_{index}_south"),
        (center_x, center_y - outer_offset, center_z),
        (inner_span + HOLE_SHAFT_WALL_THICKNESS_METERS * 2.0, HOLE_SHAFT_WALL_THICKNESS_METERS, shaft_height),
        HOLE_COLOR,
    );
    append_box_geometry(
        output,
        &format!("hole_{index}_east"),
        (center_x + outer_offset, center_y, center_z),
        (HOLE_SHAFT_WALL_THICKNESS_METERS, inner_span, shaft_height),
        HOLE_COLOR,
    );
    append_box_geometry(
        output,
        &format!("hole_{index}_west"),
        (center_x - outer_offset, center_y, center_z),
        (HOLE_SHAFT_WALL_THICKNESS_METERS, inner_span, shaft_height),
        HOLE_COLOR,
    );
}

fn append_marker(output: &mut String, name: &str, x: f64, y: f64, z: f64, color: &str) {
    let _ = writeln!(output, "    <model name=\"{name}\">");
    output.push_str("      <static>true</static>\n");
    let _ = writeln!(output, "      <pose>{x:.3} {y:.3} {z:.3} 0 0 0</pose>");
    output.push_str("      <link name=\"marker\">\n");
    output.push_str("        <visual name=\"visual\">\n");
    output.push_str("          <geometry><sphere><radius>0.18</radius></sphere></geometry>\n");
    output.push_str("          <material>\n");
    let _ = writeln!(output, "            <ambient>{color}</ambient>");
    let _ = writeln!(output, "            <diffuse>{color}</diffuse>");
    output.push_str("          </material>\n");
    output.push_str("        </visual>\n");
    output.push_str("      </link>\n");
    output.push_str("    </model>\n");
}
