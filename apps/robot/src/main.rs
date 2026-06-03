use std::collections::{HashSet, VecDeque};
use std::io::{self, BufRead};
use std::sync::{Arc, Mutex};
use std::thread;

use pathfinding::{astar, GridGraph, Node};
use shared::robot::{LidarConfig, LidarScan, NavCommand, Pose, RobotMode, RobotState};
use shared::Cave;

fn main() {
    shared::load_env();

    let pipe_mode = std::env::args().any(|a| a == "--pipe");
    let self_scan = std::env::args().any(|a| a == "--self-scan");

    let path = shared::env_or("CAVE_FILE", "cave.json".to_string());
    let json = std::fs::read_to_string(&path).unwrap();
    let cave: Cave = serde_json::from_str(&json).unwrap();
    eprintln!("loaded cave: {}×{}×{}", cave.size_x, cave.size_y, cave.size_z);
    eprintln!("start: {:?}, end: {:?}", cave.start, cave.end);

    let lidar_config = LidarConfig::default();

    let start_cell = cave.start;
    let end_cell = cave.end;

    let mut robot = Robot::new(cave, start_cell, end_cell, lidar_config);

    if pipe_mode {
        robot.run_pipe(self_scan);
    } else {
        robot.run_forward();
        robot.run_return();
    }
}

struct Robot {
    cave: Cave,
    graph: GridGraph,
    lidar_config: LidarConfig,
    pose: Pose,
    state: RobotState,
    slam: slam::Slam,
    /// EKF used during the forward phase: predicts from motor commands and
    /// accepts Gazebo pose measurements to produce a fused estimate.
    ekf: kalman::Ekf,
    explored: HashSet<(usize, usize, usize)>,
    path: Vec<Node>,
    path_index: usize,
    forward_done: bool,
    return_done: bool,
    recovery_phase: RecoveryPhase,
    best_target_dist2: f64,
    steps_no_progress: usize,
    spin_steps: usize,
    /// Ring buffer of recently visited cells for loop/oscillation detection.
    recent_cells: VecDeque<(usize, usize, usize)>,
    return_initialized: bool,
    last_cmd: NavCommand,
    /// dt of the most recent apply_command call — used for EKF prediction.
    last_dt: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum RecoveryPhase {
    None,
    Backup { remaining: usize },
    Rotate { remaining: usize },
}

impl Robot {
    fn new(
        cave: Cave,
        start: (usize, usize, usize),
        _end: (usize, usize, usize),
        lidar_config: LidarConfig,
    ) -> Self {
        let size_x = cave.size_x;
        let size_y = cave.size_y;
        let size_z = cave.size_z;

        // Full map known up-front — load all walls into the graph so A* has
        // complete information without any incremental discovery.
        let mut graph = GridGraph::new(size_x, size_y, size_z);
        for z in 0..size_z {
            for y in 0..size_y {
                for x in 0..size_x {
                    if !shared::is_passable(cave.grid[z][y][x]) {
                        graph.set_passable(Node(x, y, z), false);
                    }
                }
            }
        }

        // Weight passable cells by clearance from the nearest wall so A* routes
        // through corridor centres rather than hugging walls.
        let clearance = compute_clearance(&cave);
        for z in 0..size_z {
            for y in 0..size_y {
                for x in 0..size_x {
                    if shared::is_passable(cave.grid[z][y][x]) {
                        let cost = match clearance[z][y][x] {
                            1 => 6.0,  // adjacent to wall — strongly discouraged
                            2 => 2.5,  // one-cell buffer — mildly expensive
                            _ => 1.0,  // clear centre — free
                        };
                        graph.set_cost(Node(x, y, z), cost);
                    }
                }
            }
        }

        let (sx, sy, sz) = shared::gazebo_robot_spawn_position(start);
        let pose = Pose::new(sx, sy, sz, 0.0);

        let slam = slam::Slam::new(size_x, size_y, size_z, 200, pose);

        // EKF for forward-phase localisation: tight initial uncertainty because
        // we know the exact spawn position.
        let ekf = kalman::Ekf::new(sx, sy, sz, 0.0, 0.05, 0.01);

        let start_node = Node(cave.start.0, cave.start.1, cave.start.2);
        let end_node   = Node(cave.end.0,   cave.end.1,   cave.end.2);

        let initial_path = astar(&graph, start_node, end_node)
            .map(|(p, _)| p)
            .unwrap_or_else(|| {
                eprintln!("WARNING: no A* path from start to end at init time");
                Vec::new()
            });
        eprintln!("[ROBOT] Initial A* path: {} cells", initial_path.len());

        let mut explored = HashSet::new();
        explored.insert(start);

        Self {
            cave,
            graph,
            lidar_config,
            pose,
            state: RobotState {
                pose,
                mode: RobotMode::Forward,
                step: 0,
            },
            slam,
            ekf,
            explored,
            path: initial_path,
            path_index: 0,
            forward_done: false,
            return_done: false,
            recovery_phase: RecoveryPhase::None,
            best_target_dist2: f64::MAX,
            steps_no_progress: 0,
            spin_steps: 0,
            recent_cells: VecDeque::with_capacity(30),
            return_initialized: false,
            last_cmd: NavCommand::stop(),
            last_dt: 0.1,
        }
    }

    fn run_forward(&mut self) {
        if self.path.is_empty() {
            eprintln!("ERROR: No A* path from start to end");
            return;
        }
        let s = self.cave.start;
        let e = self.cave.end;
        eprintln!("\n===========================================");
        eprintln!("Starting");
        eprintln!("  start:  ({}, {}, {})", s.0, s.1, s.2);
        eprintln!("  end:    ({}, {}, {})", e.0, e.1, e.2);
        eprintln!("  planned steps: {}", self.path.len());
        eprintln!("===========================================\n");

        loop {
            let scan = LidarScan::simulate(self.pose, &self.cave, &self.lidar_config);
            let cmd = self.process_scan(&scan);
            self.apply_command(cmd, 0.2);

            if self.forward_done {
                break;
            }

            if self.state.step.is_multiple_of(50) {
                let pct = self.explored.len() as f64
                    / (self.cave.size_x * self.cave.size_y * self.cave.size_z) as f64
                    * 100.0;
                eprintln!(
                    "Step {}: pos=({:.1}, {:.1}, {:.1}), explored={} cells ({:.0}%)",
                    self.state.step,
                    self.pose.x,
                    self.pose.y,
                    self.pose.z,
                    self.explored.len(),
                    pct
                );
            }

            if self.state.step > 10000 {
                eprintln!("ERROR: Too many steps, aborting");
                break;
            }
        }
    }

    fn run_return(&mut self) {
        self.state.mode = RobotMode::Return;

        let slam_grid = self.slam.map.to_passable_grid();
        let slam_graph = GridGraph::from_passable_grid(&slam_grid);

        let start_node = Node(self.cave.end.0, self.cave.end.1, self.cave.end.2);
        let goal_node  = Node(self.cave.start.0, self.cave.start.1, self.cave.start.2);

        let result = astar(&slam_graph, start_node, goal_node);
        self.path = match result {
            Some((p, _)) => p,
            None => {
                eprintln!("ERROR: No return path found on SLAM map");
                return;
            }
        };

        let e = self.cave.end;
        let s = self.cave.start;
        eprintln!("\n===========================================");
        eprintln!("End reached, starting return");
        eprintln!("  end:    ({}, {}, {})", e.0, e.1, e.2);
        eprintln!("  start:  ({}, {}, {})", s.0, s.1, s.2);
        eprintln!("  planned steps: {}", self.path.len());
        eprintln!("===========================================\n");
        self.path_index = 0;

        loop {
            self.state.step += 1;

            let cell = (self.pose.x as usize, self.pose.y as usize, self.pose.z as usize);

            if cell == self.cave.start {
                eprintln!(
                    "Reached START at ({}, {}, {}) in {} total steps",
                    cell.0, cell.1, cell.2, self.state.step
                );
                break;
            }

            if self.path_index < self.path.len() {
                let target = self.path[self.path_index];
                let scan = LidarScan::simulate(self.pose, &self.cave, &self.lidar_config);
                let cmd = self.move_toward(target, &scan);
                self.apply_command(cmd, 0.2);

                let (tx, ty, tz) = shared::gazebo_cell_center((target.0, target.1, target.2));
                let dx = tx - self.pose.x;
                let dy = ty - self.pose.y;
                let dz = tz - self.pose.z;
                if dx * dx + dy * dy + dz * dz < 0.25 {
                    self.path_index += 1;
                }
            }

            if self.state.step > 15000 {
                eprintln!("ERROR: Return phase exceeded max steps");
                break;
            }
        }
    }

    fn run_pipe(&mut self, self_scan: bool) {
        if self.path.is_empty() {
            eprintln!("[ROBOT] ERROR: no A* path from start to end — check cave connectivity");
            return;
        }
        let s = self.cave.start;
        let e = self.cave.end;
        eprintln!("\n===========================================");
        eprintln!("Starting");
        eprintln!("  start:  ({}, {}, {})", s.0, s.1, s.2);
        eprintln!("  end:    ({}, {}, {})", e.0, e.1, e.2);
        eprintln!("  planned steps: {}", self.path.len());
        eprintln!("===========================================\n");

        if self_scan {
            // Shared state for real pose updates from the ROS node (via
            // the Gz pose bridge). A thread reads JSON lines from stdin and
            // stores the latest pose here so the main loop can correct its
            // dead-reckoning.
            let latest_pose: Arc<Mutex<Option<Pose>>> = Arc::new(Mutex::new(None));
            let pose_sink = Arc::clone(&latest_pose);
            thread::spawn(move || {
                let stdin = io::stdin();
                for line in stdin.lock().lines() {
                    let line = match line {
                        Ok(l) => l,
                        Err(_) => break,
                    };
                    if line.trim().is_empty() {
                        continue;
                    }
                    if let Ok(pose) = serde_json::from_str::<Pose>(&line) {
                        *pose_sink.lock().unwrap() = Some(pose);
                    }
                }
            });

            let mut last_time = std::time::Instant::now();

            // --- Forward phase ---
            loop {
                let now = std::time::Instant::now();
                let dt = (now - last_time).as_secs_f64();
                last_time = now;

                // Correct dead-reckoned pose with latest real Gazebo pose
                if let Some(p) = *latest_pose.lock().unwrap() {
                    self.pose = p;
                    self.state.pose = p;
                }

                let scan = LidarScan::simulate(self.pose, &self.cave, &self.lidar_config);
                let cmd = self.process_scan(&scan);
                self.apply_command(cmd, dt);

                let output = serde_json::json!({
                    "linear_x": cmd.linear_x,
                    "linear_y": cmd.linear_y,
                    "linear_z": cmd.linear_z,
                    "angular_z": cmd.angular_z,
                });
                eprintln!("[ROBOT] step {}: pos=({:.2},{:.2},{:.2}) yaw={:.2} path_idx={}/{} → cmd={}",
                    self.state.step,
                    self.pose.x, self.pose.y, self.pose.z,
                    self.pose.yaw,
                    self.path_index, self.path.len(),
                    output);
                println!("{output}");

                if self.forward_done {
                    break;
                }

                if self.state.step > 20000 {
                    eprintln!("[ROBOT] Pipe mode: max steps reached, exiting");
                    return;
                }

                std::thread::sleep(std::time::Duration::from_millis(100));
            }

            // --- Return phase ---
            let start_cell = self.cave.start;
            let current_cell = (self.pose.x as usize, self.pose.y as usize, self.pose.z as usize);

            if current_cell == start_cell {
                eprintln!("[ROBOT] Already at start, skipping return phase");
                return;
            }

            eprintln!("[ROBOT] Computing return path from ({},{},{}) to ({},{},{}) on known map…",
                current_cell.0, current_cell.1, current_cell.2,
                start_cell.0, start_cell.1, start_cell.2);

            let goal_node  = Node(start_cell.0,   start_cell.1,   start_cell.2);
            let start_node = Node(current_cell.0, current_cell.1, current_cell.2);

            let return_path = match pathfinding::astar(&self.graph, start_node, goal_node) {
                Some((p, _)) => p,
                None => {
                    eprintln!("[ROBOT] No return path found on known map — staying put");
                    let output = serde_json::json!({"linear_x":0.0,"linear_y":0.0,"linear_z":0.0,"angular_z":0.0});
                    println!("{output}");
                    return;
                }
            };

            self.path = return_path;
            self.path_index = 0;
            self.state.mode = RobotMode::Return;
            self.spin_steps = 0;
            self.recent_cells.clear();
            eprintln!("[ROBOT] Return path: {} cells — switching to BLIND navigation (EKF dead-reckoning, no LiDAR)",
                self.path.len());

            // Initialise EKF from current pose.  We know our position exactly at
            // handoff (we just arrived at the end cell), so start with tight uncertainty.
            let mut ekf = kalman::Ekf::new(
                self.pose.x, self.pose.y, self.pose.z, self.pose.yaw,
                0.05,  // 5 cm initial position std
                0.01,  // ~0.6° initial heading std
            );

            loop {
                self.state.step += 1;

                let now = std::time::Instant::now();
                let dt = (now - last_time).as_secs_f64().clamp(0.01, 0.5);
                last_time = now;

                // In blind mode we deliberately ignore the Gazebo pose bridge
                // and drive purely from the EKF estimate.
                let cell = (self.pose.x as usize, self.pose.y as usize, self.pose.z as usize);

                if cell == start_cell {
                    eprintln!("[ROBOT] Reached START at ({},{},{}) in {} total steps",
                        cell.0, cell.1, cell.2, self.state.step);
                    break;
                }

                if self.path_index >= self.path.len() {
                    eprintln!("[ROBOT] Return path exhausted at ({:.2},{:.2},{:.2}) — replanning…",
                        self.pose.x, self.pose.y, self.pose.z);
                    let result = pathfinding::astar(&self.graph, Node(cell.0, cell.1, cell.2), goal_node);
                    match result {
                        Some((p, _)) => {
                            self.path = p;
                            self.path_index = 0;
                        }
                        None => {
                            eprintln!("[ROBOT] Replan failed, stopping");
                            let output = serde_json::json!({"linear_x":0.0,"linear_y":0.0,"linear_z":0.0,"angular_z":0.0});
                            println!("{output}");
                            break;
                        }
                    }
                }

                // BLIND navigation — no LiDAR, no SLAM update.
                // Use move_toward_blind and track state via EKF.
                let target = self.path[self.path_index];
                let cmd = self.move_toward_blind(target);

                // Spin detection for blind phase
                let is_spinning = cmd.linear_x.abs() < 0.01 && cmd.angular_z.abs() > 0.01;
                if is_spinning {
                    self.spin_steps += 1;
                } else {
                    self.spin_steps = 0;
                }

                self.apply_command(cmd, dt);
                ekf.predict(cmd.linear_x, cmd.linear_z, cmd.angular_z, dt);

                // Keep self.pose in sync with EKF state
                self.pose.x   = ekf.x();
                self.pose.y   = ekf.y();
                self.pose.z   = ekf.z();
                self.pose.yaw = ekf.yaw();
                self.state.pose = self.pose;

                let (tx, ty, tz) = shared::gazebo_cell_center((target.0, target.1, target.2));
                let dx = tx - self.pose.x;
                let dy = ty - self.pose.y;
                let dz = tz - self.pose.z;
                let dist2 = dx * dx + dy * dy + dz * dz;
                if dist2 < 0.25 {
                    self.path_index += 1;
                    self.spin_steps = 0;
                }

                // Back off if spinning in place
                if self.spin_steps >= 20 {
                    eprintln!("[ROBOT] step {} [BLIND] spinning for {} steps — injecting backup",
                        self.state.step, self.spin_steps);
                    self.spin_steps = 0;
                    // Issue a short reverse burst directly (no recovery_phase in blind mode)
                    let backup = NavCommand { linear_x: -0.4, linear_y: 0.0, linear_z: 0.0, angular_z: 0.0 };
                    for _ in 0..8 {
                        self.apply_command(backup, 0.1);
                        ekf.predict(backup.linear_x, backup.linear_z, backup.angular_z, 0.1);
                        let output = serde_json::json!({
                            "linear_x": backup.linear_x, "linear_y": 0.0, "linear_z": 0.0, "angular_z": 0.0,
                        });
                        println!("{output}");
                        std::thread::sleep(std::time::Duration::from_millis(100));
                    }
                    self.pose.x   = ekf.x();
                    self.pose.y   = ekf.y();
                    self.pose.z   = ekf.z();
                    self.pose.yaw = ekf.yaw();
                }

                let action = if cmd.linear_z.abs() > 0.01 {
                    format!("CLIMB  lz={:+.1}", cmd.linear_z)
                } else if cmd.angular_z.abs() > 0.01 {
                    format!("ROTATE az={:+.2}", cmd.angular_z)
                } else if cmd.linear_x.abs() > 0.01 {
                    format!("FORWARD lx={:.1}", cmd.linear_x)
                } else {
                    "STOP".to_string()
                };

                eprintln!("[ROBOT] step {:4} [BLIND] ekf=({:.2},{:.2},{:.2}) yaw={:.2} σ={:.3}m | {} | target=({},{},{}) dist={:.2} | path {}/{}",
                    self.state.step,
                    ekf.x(), ekf.y(), ekf.z(), ekf.yaw(),
                    ekf.position_std(),
                    action,
                    target.0, target.1, target.2, dist2.sqrt(),
                    self.path_index, self.path.len(),
                );

                let output = serde_json::json!({
                    "linear_x": cmd.linear_x,
                    "linear_y": cmd.linear_y,
                    "linear_z": cmd.linear_z,
                    "angular_z": cmd.angular_z,
                });
                println!("{output}");

                if self.state.step > 30000 {
                    eprintln!("[ROBOT] Max steps reached, exiting");
                    break;
                }

                std::thread::sleep(std::time::Duration::from_millis(100));
            }

            eprintln!("[ROBOT] Return phase complete!");
        } else {
            let stdin = io::stdin();
            let reader = stdin.lock();
            for line in reader.lines() {
                let line = match line {
                    Ok(l) => l,
                    Err(e) => {
                        eprintln!("[ROBOT] pipe read error: {e}");
                        break;
                    }
                };

                if line.trim().is_empty() {
                    eprintln!("[ROBOT] skipping empty line");
                    continue;
                }

                let scan: LidarScan = match serde_json::from_str(&line) {
                    Ok(s) => s,
                    Err(e) => {
                        eprintln!("[ROBOT] pipe parse error: {e}");
                        continue;
                    }
                };

                // Reckon movement from the PREVIOUS command (which Gazebo
                // applied during the 0.1 s between this scan and the last).
                if scan.pose.is_none() {
                    self.apply_command(self.last_cmd, 0.1);
                }

                let cmd = self.process_scan(&scan);

                self.last_cmd = cmd;
                let output = serde_json::json!({
                    "linear_x": cmd.linear_x,
                    "linear_y": cmd.linear_y,
                    "linear_z": cmd.linear_z,
                    "angular_z": cmd.angular_z,
                });
                println!("{output}");

                if self.return_done {
                    eprintln!("[ROBOT] Return phase complete — exiting pipe mode");
                    break;
                }

                if self.state.step > 20000 {
                    eprintln!("[ROBOT] Pipe mode: max steps reached, exiting");
                    break;
                }
            }
        }
        eprintln!("[ROBOT] pipe mode exiting");
    }

    fn skip_occupied_cells(&mut self) {
        while self.path_index < self.path.len() {
            let t = self.path[self.path_index];
            let cx = self.pose.x as usize;
            let cy = self.pose.y as usize;
            let cz = self.pose.z as usize;
            if (t.0, t.1, t.2) != (cx, cy, cz) || self.path_index == self.path.len() - 1 {
                break;
            }
            self.path_index += 1;
        }
    }

    fn process_scan(&mut self, scan: &LidarScan) -> NavCommand {
        self.state.step += 1;

        // EKF is advanced in apply_command alongside dead-reckoning.
        // Here we only do the measurement update when Gazebo pose is available,
        // fusing it with the accumulated dead-reckoning to correct drift.
        let pose_source = if let Some(p) = scan.pose {
            self.ekf.update(p.x, p.y, p.z, p.yaw, 0.05, 0.02);
            let fused = Pose::new(self.ekf.x(), self.ekf.y(), self.ekf.z(), self.ekf.yaw());
            self.pose = fused;
            self.state.pose = fused;
            "ekf+gazebo"
        } else {
            "dead-reckoning"
        };

        // Use the accurate EKF/Gazebo pose for map updates so the SLAM
        // occupancy grid is correct regardless of particle filter drift.
        self.slam.update_with_map_pose(scan, self.pose);

        let cell = (self.pose.x as usize, self.pose.y as usize, self.pose.z as usize);
        self.explored.insert(cell);

        if !self.forward_done {

            {
                let end = self.cave.end;
                let (ex, ey, _) = shared::gazebo_cell_center((end.0, end.1, end.2));
                let edx = ex - self.pose.x;
                let edy = ey - self.pose.y;
                // Use the same 0.5 m radius used for waypoint advancement so the
                // end detection is consistent with how every other cell is "reached".
                if edx * edx + edy * edy < 0.25 {
                    self.forward_done = true;
                    return NavCommand::stop();
                }
            }

            self.skip_occupied_cells();

            if self.path_index < self.path.len() {
                let target = self.path[self.path_index];

                let (tx, ty, tz) = shared::gazebo_cell_center((target.0, target.1, target.2));
                let dx = tx - self.pose.x;
                let dy = ty - self.pose.y;
                let dz = tz - self.pose.z;
                let dist2 = dx * dx + dy * dy + dz * dz;
                let path_just_advanced = dist2 < 0.25;
                if path_just_advanced {
                    self.path_index += 1;
                    self.best_target_dist2 = f64::MAX;
                    self.steps_no_progress = 0;
                }

                // Forward navigation: recovery overrides normal path-following.
                // Reactive LiDAR avoidance is intentionally omitted — the
                // clearance-weighted A* path already routes through corridor
                // centres, so the path itself provides the obstacle margin.
                let mut cmd = if self.recovery_phase != RecoveryPhase::None {
                    self.recovery_step()
                } else {
                    self.move_toward_blind(target)
                };

                if self.recovery_phase == RecoveryPhase::None && !path_just_advanced {
                    let is_spinning = cmd.linear_x.abs() < 0.01 && cmd.angular_z.abs() > 0.01;
                    if is_spinning {
                        self.spin_steps += 1;
                        if self.spin_steps >= 40 {
                            eprintln!("[ROBOT] step {}: spinning for {} steps — backing up + replanning",
                                self.state.step, self.spin_steps);
                            self.spin_steps = 0;
                            self.steps_no_progress = 0;
                            self.recovery_phase = RecoveryPhase::Backup { remaining: 12 };
                            cmd = self.recovery_step();
                            self.replan_forward(cell);
                        }
                    } else {
                        self.spin_steps = 0;
                        if self.best_target_dist2 == f64::MAX || dist2 < self.best_target_dist2 - 0.01 {
                            self.best_target_dist2 = dist2;
                            self.steps_no_progress = 0;
                        } else {
                            self.steps_no_progress += 1;
                            if self.steps_no_progress >= 30 {
                                // Don't back up — just skip the stuck waypoint and
                                // replan. Backing up moves the drone away from a target
                                // it might be close to, making things worse.
                                eprintln!("[ROBOT] step {}: no progress for {} steps (dist={:.2}) — skipping waypoint + replanning",
                                    self.state.step, self.steps_no_progress, dist2.sqrt());
                                self.steps_no_progress = 0;
                                self.best_target_dist2 = f64::MAX;
                                if self.path_index + 1 < self.path.len() {
                                    self.path_index += 1;
                                }
                                self.replan_forward(cell);
                            }
                        }
                    }
                }

                let action = if cmd.linear_z.abs() > 0.01 {
                    format!("CLIMB  lz={:+.1}", cmd.linear_z)
                } else if cmd.angular_z.abs() > 0.01 {
                    format!("ROTATE az={:+.2}", cmd.angular_z)
                } else if cmd.linear_x.abs() > 0.01 {
                    format!("FORWARD lx={:.1}", cmd.linear_x)
                } else {
                    "STOP".to_string()
                };

                eprintln!(
                    "[ROBOT] step {:4} | pose=({:.2},{:.2},{:.2}) yaw={:.2} [{}] | {} | target=({},{},{}) dist={:.2} | path {}/{}",
                    self.state.step,
                    self.pose.x, self.pose.y, self.pose.z, self.pose.yaw,
                    pose_source,
                    action,
                    target.0, target.1, target.2, dist2.sqrt(),
                    self.path_index, self.path.len(),
                );

                return cmd;
            }

            // Path exhausted.  If the planned path ended at the end cell the drone
            // advanced past it within 0.5 m — that's the same "reached" standard
            // used for every waypoint, so declare success.
            let end = self.cave.end;
            let end_node = pathfinding::Node(end.0, end.1, end.2);
            if self.path.last() == Some(&end_node) {
                eprintln!(
                    "[ROBOT] step {}: path led to end cell and was exhausted — forward done",
                    self.state.step
                );
                self.forward_done = true;
                return NavCommand::stop();
            }
            // Path didn't end at the goal (shouldn't happen normally) — home directly.
            eprintln!(
                "[ROBOT] step {:4} | pose=({:.2},{:.2},{:.2}) [{}] | path exhausted → homing to end ({},{},{})",
                self.state.step,
                self.pose.x, self.pose.y, self.pose.z,
                pose_source,
                end.0, end.1, end.2,
            );
            return self.move_toward(end_node, scan);
        }

        // After forward phase, switch to return phase
        if self.forward_done {
            return self.process_return_scan(scan);
        }

        NavCommand::stop()
    }

    fn process_return_scan(&mut self, scan: &LidarScan) -> NavCommand {
        // Keep refining the SLAM map during return (pose is NOT taken from
        // SLAM — the particle filter never runs predict so it stays at spawn;
        // we use self.pose = accurate EKF/Gazebo pose throughout).
        self.slam.update(scan);

        if !self.return_initialized {
            self.return_initialized = true;
            self.state.mode = RobotMode::Return;

            // Plan the return path on the SLAM-built occupancy map.
            // Use self.pose (EKF/Gazebo) for the starting cell — not
            // slam.estimated_pose() which is stuck at the spawn position.
            let slam_grid = self.slam.map.to_passable_grid();
            let slam_graph = GridGraph::from_passable_grid(&slam_grid);

            let start_node   = Node(self.cave.start.0, self.cave.start.1, self.cave.start.2);
            let current_cell = (self.pose.x as usize, self.pose.y as usize, self.pose.z as usize);
            let current_node = Node(current_cell.0, current_cell.1, current_cell.2);

            self.path = match astar(&slam_graph, current_node, start_node) {
                Some((p, _)) => p,
                None => {
                    eprintln!("[ROBOT] SLAM path failed, falling back to known map");
                    let gaz_cell = (self.pose.x as usize, self.pose.y as usize, self.pose.z as usize);
                    let gaz_node = Node(gaz_cell.0, gaz_cell.1, gaz_cell.2);
                    match pathfinding::astar(&self.graph, gaz_node, start_node) {
                        Some((p, _)) => p,
                        None => {
                            eprintln!("[ROBOT] No return path found — stopping");
                            return NavCommand::stop();
                        }
                    }
                }
            };
            self.path_index = 0;

            let s = self.cave.start;
            eprintln!("\n===========================================");
            eprintln!("End reached, starting return");
            eprintln!("  end pos:  ({:.2}, {:.2}, {:.2})", self.pose.x, self.pose.y, self.pose.z);
            eprintln!("  start:    ({}, {}, {})", s.0, s.1, s.2);
            eprintln!("  planned steps: {}", self.path.len());
            eprintln!("===========================================\n");
        }

        let cell = (self.pose.x as usize, self.pose.y as usize, self.pose.z as usize);

        let (sx, sy, _) = shared::gazebo_cell_center(self.cave.start);
        let sdx = sx - self.pose.x;
        let sdy = sy - self.pose.y;
        if sdx * sdx + sdy * sdy < 0.25 || cell == self.cave.start {
            if !self.return_done {
                self.return_done = true;
                eprintln!(
                    "[ROBOT] Reached START at ({},{},{}) in {} total steps",
                    cell.0, cell.1, cell.2, self.state.step
                );
            }
            return NavCommand::stop();
        }

        if self.path_index >= self.path.len() {
            eprintln!("[ROBOT] Return path exhausted — replanning on SLAM map …");
            let start_node   = Node(self.cave.start.0, self.cave.start.1, self.cave.start.2);
            let current_node = Node(cell.0, cell.1, cell.2);
            let slam_grid  = self.slam.map.to_passable_grid();
            let slam_graph = GridGraph::from_passable_grid(&slam_grid);
            self.path = match astar(&slam_graph, current_node, start_node) {
                Some((p, _)) => p,
                None => {
                    eprintln!("[ROBOT] SLAM replan failed, trying known map …");
                    match pathfinding::astar(&self.graph, current_node, start_node) {
                        Some((p, _)) => p,
                        None => {
                            eprintln!("[ROBOT] Return replan failed — stopping");
                            return NavCommand::stop();
                        }
                    }
                }
            };
            self.path_index = 0;
        }

        let target = self.path[self.path_index];
        // Return phase uses pure heading control — no LiDAR-based avoidance.
        // The A* path on the known map is already wall-free, and the 2D LaserScan
        // coming from the ROS bridge has vertical angles hardcoded to 0 so it
        // can't be trusted for reactive avoidance anyway.
        let mut cmd = if self.recovery_phase != RecoveryPhase::None {
            self.recovery_step()
        } else {
            self.move_toward_blind(target)
        };

        let (tx, ty, tz) = shared::gazebo_cell_center((target.0, target.1, target.2));
        let dx = tx - self.pose.x;
        let dy = ty - self.pose.y;
        let dz = tz - self.pose.z;
        let dist2 = dx * dx + dy * dy + dz * dz;
        let path_just_advanced = dist2 < 0.25;
        if path_just_advanced {
            self.path_index += 1;
            self.best_target_dist2 = f64::MAX;
            self.steps_no_progress = 0;
        }

        if self.recovery_phase == RecoveryPhase::None && !path_just_advanced {
            let cell = (self.pose.x as usize, self.pose.y as usize, self.pose.z as usize);
            let is_spinning = cmd.linear_x.abs() < 0.01 && cmd.angular_z.abs() > 0.01;
            if is_spinning {
                self.spin_steps += 1;
                if self.spin_steps >= 40 {
                    eprintln!("[ROBOT] step {} [RETURN] spinning — backing up + replanning",
                        self.state.step);
                    self.spin_steps = 0;
                    self.recovery_phase = RecoveryPhase::Backup { remaining: 12 };
                    cmd = self.recovery_step();
                    self.replan_return(cell);
                }
            } else {
                self.spin_steps = 0;
                if self.best_target_dist2 == f64::MAX || dist2 < self.best_target_dist2 - 0.01 {
                    self.best_target_dist2 = dist2;
                    self.steps_no_progress = 0;
                } else {
                    self.steps_no_progress += 1;
                    if self.steps_no_progress >= 30 {
                        eprintln!("[ROBOT] step {} [RETURN] no progress (dist={:.2}) — skipping waypoint + replanning",
                            self.state.step, dist2.sqrt());
                        self.steps_no_progress = 0;
                        self.best_target_dist2 = f64::MAX;
                        if self.path_index + 1 < self.path.len() {
                            self.path_index += 1;
                        }
                        self.replan_return(cell);
                    }
                }
            }
        }

        let action = if cmd.linear_z.abs() > 0.01 {
            format!("CLIMB  lz={:+.1}", cmd.linear_z)
        } else if cmd.angular_z.abs() > 0.01 {
            format!("ROTATE az={:+.2}", cmd.angular_z)
        } else if cmd.linear_x.abs() > 0.01 {
            format!("FORWARD lx={:.1}", cmd.linear_x)
        } else {
            "STOP".to_string()
        };

        eprintln!(
            "[ROBOT] step {:4} [RETURN] pose=({:.2},{:.2},{:.2}) yaw={:.2} [gazebo] | {} | target=({},{},{}) dist={:.2} | path {}/{}",
            self.state.step,
            self.pose.x, self.pose.y, self.pose.z, self.pose.yaw,
            action,
            target.0, target.1, target.2,
            dist2.sqrt(),
            self.path_index, self.path.len(),
        );

        cmd
    }

    fn apply_command(&mut self, cmd: NavCommand, dt: f64) {
        self.last_dt = dt;
        self.last_cmd = cmd;
        self.ekf.predict(cmd.linear_x, cmd.linear_z, cmd.angular_z, dt);
        self.pose.x += cmd.linear_x * self.pose.yaw.cos() * dt;
        self.pose.y += cmd.linear_x * self.pose.yaw.sin() * dt;
        self.pose.z += cmd.linear_z * dt;
        self.pose.yaw = ang_diff(self.pose.yaw + cmd.angular_z * dt, 0.0);
        self.state.pose = self.pose;
    }

    /// Recompute the forward path with A* from the given cell.
    /// Called after a recovery backup so the drone gets a fresh route from
    /// wherever it ended up.
    fn replan_forward(&mut self, cell: (usize, usize, usize)) {
        let current_node = Node(cell.0, cell.1, cell.2);
        let end_node = Node(self.cave.end.0, self.cave.end.1, self.cave.end.2);
        match astar(&self.graph, current_node, end_node) {
            Some((p, _)) => {
                eprintln!("[ROBOT] replanned forward: {} cells from ({},{},{})",
                    p.len(), cell.0, cell.1, cell.2);
                self.path = p;
                self.path_index = 0;
                self.best_target_dist2 = f64::MAX;
                self.steps_no_progress = 0;
            }
            None => {
                eprintln!("[ROBOT] replan failed — no A* path from ({},{},{})", cell.0, cell.1, cell.2);
            }
        }
    }

    fn replan_return(&mut self, cell: (usize, usize, usize)) {
        let current_node = Node(cell.0, cell.1, cell.2);
        let start_node = Node(self.cave.start.0, self.cave.start.1, self.cave.start.2);
        match astar(&self.graph, current_node, start_node) {
            Some((p, _)) => {
                eprintln!("[ROBOT] replanned return: {} cells from ({},{},{})",
                    p.len(), cell.0, cell.1, cell.2);
                self.path = p;
                self.path_index = 0;
                self.best_target_dist2 = f64::MAX;
                self.steps_no_progress = 0;
            }
            None => {
                eprintln!("[ROBOT] return replan failed from ({},{},{})", cell.0, cell.1, cell.2);
            }
        }
    }

    fn recovery_step(&mut self) -> NavCommand {
        match self.recovery_phase {
            RecoveryPhase::Backup { remaining } => {
                let cmd = NavCommand {
                    linear_x: -0.4,
                    linear_y: 0.0,
                    linear_z: 0.0,
                    angular_z: 0.0,
                };
                if remaining <= 1 {
                    self.recovery_phase = RecoveryPhase::Rotate { remaining: 10 };
                } else {
                    self.recovery_phase = RecoveryPhase::Backup { remaining: remaining - 1 };
                }
                cmd
            }
            RecoveryPhase::Rotate { remaining } => {
                let cmd = NavCommand {
                    linear_x: 0.0,
                    linear_y: 0.0,
                    linear_z: 0.0,
                    angular_z: 0.5,
                };
                if remaining <= 1 {
                    self.recovery_phase = RecoveryPhase::None;
                } else {
                    self.recovery_phase = RecoveryPhase::Rotate { remaining: remaining - 1 };
                }
                cmd
            }
            RecoveryPhase::None => NavCommand::stop(),
        }
    }

    /// Proportional heading controller — face the waypoint then drive.
    fn move_toward(&self, target: Node, _scan: &LidarScan) -> NavCommand {
        let (tx, ty, tz) = shared::gazebo_cell_center((target.0, target.1, target.2));
        let dz = tz - self.pose.z;

        if (self.pose.z as usize) != target.2 || dz.abs() > 0.3 {
            return NavCommand {
                linear_x: 0.0,
                linear_y: 0.0,
                linear_z: dz.signum() * 1.0,
                angular_z: 0.0,
            };
        }

        let dx = tx - self.pose.x;
        let dy = ty - self.pose.y;
        let heading_err = ang_diff(dy.atan2(dx), self.pose.yaw);

        if heading_err.abs() > 0.35 {
            NavCommand {
                linear_x: 0.0,
                linear_y: 0.0,
                linear_z: 0.0,
                angular_z: heading_err.signum() * 1.0,
            }
        } else {
            NavCommand {
                linear_x: 0.8,
                linear_y: 0.0,
                linear_z: 0.0,
                angular_z: heading_err * 2.5,
            }
        }
    }

    fn move_toward_blind(&self, target: Node) -> NavCommand {
        let (tx, ty, tz) = shared::gazebo_cell_center((target.0, target.1, target.2));
        let dz = tz - self.pose.z;

        if (self.pose.z as usize) != target.2 || dz.abs() > 0.3 {
            return NavCommand {
                linear_x: 0.0,
                linear_y: 0.0,
                linear_z: dz.signum() * 1.0,
                angular_z: 0.0,
            };
        }

        let dx = tx - self.pose.x;
        let dy = ty - self.pose.y;
        let heading_err = ang_diff(dy.atan2(dx), self.pose.yaw);

        if heading_err.abs() > 0.35 {
            NavCommand {
                linear_x: 0.0,
                linear_y: 0.0,
                linear_z: 0.0,
                angular_z: heading_err.signum() * 1.0,
            }
        } else {
            NavCommand {
                linear_x: 0.8,
                linear_y: 0.0,
                linear_z: 0.0,
                angular_z: heading_err * 2.5,
            }
        }
    }
}

/// BFS flood-fill from wall cells to compute, for each passable cell,
/// its Manhattan distance to the nearest wall in the x-y plane.
/// Used to bias A* toward corridor centres.
fn compute_clearance(cave: &Cave) -> Vec<Vec<Vec<usize>>> {
    use std::collections::VecDeque;
    let mut dist = vec![vec![vec![usize::MAX; cave.size_x]; cave.size_y]; cave.size_z];
    let mut queue = VecDeque::new();

    for z in 0..cave.size_z {
        for y in 0..cave.size_y {
            for x in 0..cave.size_x {
                if !shared::is_passable(cave.grid[z][y][x]) {
                    dist[z][y][x] = 0;
                    queue.push_back((x, y, z));
                }
            }
        }
    }

    while let Some((x, y, z)) = queue.pop_front() {
        let d = dist[z][y][x];
        for (dx, dy) in [(-1i64, 0i64), (1, 0), (0, -1), (0, 1)] {
            let nx = x as i64 + dx;
            let ny = y as i64 + dy;
            if nx < 0 || ny < 0 { continue; }
            let (nx, ny) = (nx as usize, ny as usize);
            if nx >= cave.size_x || ny >= cave.size_y { continue; }
            if dist[z][ny][nx] == usize::MAX {
                dist[z][ny][nx] = d + 1;
                queue.push_back((nx, ny, z));
            }
        }
    }
    dist
}

fn ang_diff(a: f64, b: f64) -> f64 {
    let mut d = a - b;
    while d > std::f64::consts::PI {
        d -= 2.0 * std::f64::consts::PI;
    }
    while d < -std::f64::consts::PI {
        d += 2.0 * std::f64::consts::PI;
    }
    d
}
