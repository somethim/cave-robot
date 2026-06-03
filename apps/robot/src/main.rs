use std::collections::{HashSet, VecDeque};
use std::io::{self, BufRead};
use std::sync::{Arc, Mutex};
use std::thread;

use pathfinding::{astar, DStarLite, GridGraph, Node};
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
    explored: HashSet<(usize, usize, usize)>,
    planner: DStarLite,
    path: Vec<Node>,
    path_index: usize,
    forward_done: bool,
    end_cell_steps: usize,
    wall_stuck_count: usize,
    recovery_phase: RecoveryPhase,
    in_avoidance: bool,
    avoid_steps_remaining: usize,
    best_target_dist2: f64,
    steps_no_progress: usize,
    /// Counts consecutive steps of pure rotation with no forward movement.
    /// Triggers a backup recovery when it exceeds the threshold.
    spin_steps: usize,
    /// Ring buffer of recently visited cells for loop/oscillation detection.
    recent_cells: VecDeque<(usize, usize, usize)>,
    return_initialized: bool,
    last_cmd: NavCommand,
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

        // Start with a fully-open graph — D* Lite assumes everything is
        // passable until the LiDAR proves otherwise.  Walls are added
        // incrementally via discover_obstacles() as the drone explores.
        let graph = GridGraph::new(size_x, size_y, size_z);

        let (sx, sy, sz) = shared::gazebo_robot_spawn_position(start);
        let pose = Pose::new(sx, sy, sz, 0.0);

        let slam = slam::Slam::new(size_x, size_y, size_z, 200, pose);

        let start_node = Node(cave.start.0, cave.start.1, cave.start.2);
        let end_node = Node(cave.end.0, cave.end.1, cave.end.2);
        let planner = DStarLite::new(graph.clone(), start_node, end_node);

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
            explored,
            planner,
            path: Vec::new(),
            path_index: 0,
            forward_done: false,
            end_cell_steps: 0,
            wall_stuck_count: 0,
            recovery_phase: RecoveryPhase::None,
            in_avoidance: false,
            avoid_steps_remaining: 0,
            best_target_dist2: f64::MAX,
            steps_no_progress: 0,
            spin_steps: 0,
            recent_cells: VecDeque::with_capacity(30),
            return_initialized: false,
            last_cmd: NavCommand::stop(),
        }
    }

    fn run_forward(&mut self) {
        eprintln!("\n=== FORWARD PHASE: Start → End ===");

        let init_path = self.planner.get_path();
        if init_path.is_none() {
            eprintln!("ERROR: No initial path from start to end");
            return;
        }

        self.path = init_path.unwrap();
        eprintln!("Initial path length: {} cells", self.path.len());
        self.path_index = 0;

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
        eprintln!("\n=== RETURN PHASE: End → Start ===");

        self.state.mode = RobotMode::Return;

        let slam_grid = self.slam.map.to_passable_grid();
        let slam_graph = GridGraph::from_passable_grid(&slam_grid);

        let start_node = Node(self.cave.end.0, self.cave.end.1, self.cave.end.2);
        let goal_node = Node(self.cave.start.0, self.cave.start.1, self.cave.start.2);

        let sg = self.cave.start;
        eprintln!(
            "slam map: start_cell=({},{},{}) passable={} prob={}",
            sg.0, sg.1, sg.2,
            slam_graph.is_passable(goal_node),
            self.slam.map.probability(sg.0, sg.1, sg.2),
        );
        eprintln!(
            "slam map: end_cell=({},{},{}) passable={} prob={}",
            self.cave.end.0, self.cave.end.1, self.cave.end.2,
            slam_graph.is_passable(start_node),
            self.slam.map.probability(self.cave.end.0, self.cave.end.1, self.cave.end.2),
        );

        let result = astar(&slam_graph, start_node, goal_node);
        self.path = match result {
            Some((p, _)) => p,
            None => {
                eprintln!("ERROR: No return path found on SLAM map");
                return;
            }
        };

        eprintln!("Return path length: {} cells", self.path.len());
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
        eprintln!("[ROBOT] computing initial path from ({},{},{}) to ({},{},{})…",
            self.cave.start.0, self.cave.start.1, self.cave.start.2,
            self.cave.end.0, self.cave.end.1, self.cave.end.2);
        let init_path = self.planner.get_path();
        if init_path.is_none() {
            eprintln!("[ROBOT] ERROR: no path found from start to end — check cave connectivity");
            return;
        }
        self.path = init_path.unwrap();
        self.path_index = 0;
        eprintln!("[ROBOT] pipe mode started | path {} cells | initial pose ({:.2},{:.2},{:.2}) yaw={:.2}",
            self.path.len(), self.pose.x, self.pose.y, self.pose.z, self.pose.yaw);
        eprintln!("[ROBOT] path preview (first 5): {:?}", &self.path[..self.path.len().min(5)]);

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
                    eprintln!("[ROBOT] Forward phase complete!");
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

            eprintln!("[ROBOT] Computing return path from ({},{},{}) to ({},{},{}) via SLAM map…",
                current_cell.0, current_cell.1, current_cell.2,
                start_cell.0, start_cell.1, start_cell.2);

            let slam_grid = self.slam.map.to_passable_grid();
            let slam_graph = GridGraph::from_passable_grid(&slam_grid);

            let goal_node = Node(start_cell.0, start_cell.1, start_cell.2);
            let start_node = Node(current_cell.0, current_cell.1, current_cell.2);

            let result = pathfinding::astar(&slam_graph, start_node, goal_node);
            let return_path = match result {
                Some((p, _)) => p,
                None => {
                    eprintln!("[ROBOT] No return path found on SLAM map — staying put");
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
                    let result = pathfinding::astar(&slam_graph, Node(cell.0, cell.1, cell.2), goal_node);
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

                if self.forward_done {
                    eprintln!("[ROBOT] Forward phase complete!");
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

        // If the scan carries the actual pose (from the ROS node), use it
        // directly instead of dead-reckoning, so navigation stays accurate.
        let pose_source = if let Some(p) = scan.pose {
            self.pose = p;
            self.state.pose = p;
            "gazebo"
        } else {
            "dead-reckoning"
        };

        self.slam.update(scan);

        let cell = (self.pose.x as usize, self.pose.y as usize, self.pose.z as usize);
        self.explored.insert(cell);

        if !self.forward_done {
            let discovered_new = self.discover_obstacles(scan);

            if discovered_new {
                // Only reset the path when the remaining route is actually
                // blocked.  Replanning on every new discovery (which happens
                // nearly every step with a 20 m LiDAR) throws away path_index
                // progress and makes the robot restart from the current cell
                // each time, causing it to loop back to the same waypoints.
                let remaining_blocked = self.path_index >= self.path.len()
                    || self.path[self.path_index..]
                        .iter()
                        .any(|n| !self.graph.is_passable(*n));

                if remaining_blocked {
                    let current_node = Node(cell.0, cell.1, cell.2);
                    self.planner.move_start(current_node);
                    self.path = match self.planner.get_path() {
                        Some(p) => p,
                        None => {
                            eprintln!("[ROBOT] step {}: NO PATH after replanning! pos=({:.2},{:.2},{:.2})",
                                self.state.step, self.pose.x, self.pose.y, self.pose.z);
                            return NavCommand::stop();
                        }
                    };
                    self.path_index = 0;
                    eprintln!("[ROBOT] step {}: path blocked — replanned {} cells",
                        self.state.step, self.path.len());
                }
            }

            {
                let end = self.cave.end;
                let cell = (self.pose.x as usize, self.pose.y as usize, self.pose.z as usize);
                if cell == (end.0, end.1, end.2) {
                    self.end_cell_steps += 1;
                    if self.end_cell_steps >= 5 {
                        eprintln!(
                            "[ROBOT] step {}: reached END ({},{},{}) in {} steps",
                            self.state.step, end.0, end.1, end.2, self.state.step
                        );
                        self.forward_done = true;
                        return NavCommand::stop();
                    }
                } else {
                    self.end_cell_steps = 0;
                }
            }

            self.skip_occupied_cells();

            if self.path_index < self.path.len() {
                let target = self.path[self.path_index];
                let mut cmd = self.move_toward(target, scan);

                let (tx, ty, tz) = shared::gazebo_cell_center((target.0, target.1, target.2));
                let dx = tx - self.pose.x;
                let dy = ty - self.pose.y;
                let dz = tz - self.pose.z;
                let dist2 = dx * dx + dy * dy + dz * dz;
                if dist2 < 0.25 {
                    self.path_index += 1;
                    self.best_target_dist2 = f64::MAX;
                    self.steps_no_progress = 0;
                }

                if self.recovery_phase != RecoveryPhase::None {
                    cmd = self.recovery_step();
                } else if self.wall_too_close(scan) {
                    self.avoid_steps_remaining = 5;
                    if !self.in_avoidance {
                        self.wall_stuck_count += 1;
                        if self.wall_stuck_count >= 4 {
                            self.recovery_phase = RecoveryPhase::Backup { remaining: 8 };
                            cmd = self.recovery_step();
                        } else {
                            self.in_avoidance = true;
                            cmd = self.avoid_obstacle(scan);
                        }
                    } else {
                        cmd = self.avoid_obstacle(scan);
                    }
                } else if self.in_avoidance {
                    self.avoid_steps_remaining -= 1;
                    if self.avoid_steps_remaining == 0 {
                        self.in_avoidance = false;
                        self.wall_stuck_count = 0;
                    } else {
                        cmd = self.avoid_obstacle(scan);
                    }
                } else {
                    self.wall_stuck_count = 0;
                }

                if self.recovery_phase == RecoveryPhase::None {
                    // Track pure-spin steps separately — they don't advance dist2
                    // so the standard no-progress counter never catches an infinite spin.
                    let is_spinning = cmd.linear_x.abs() < 0.01 && cmd.angular_z.abs() > 0.01;
                    if is_spinning {
                        self.spin_steps += 1;
                        if self.spin_steps >= 20 {
                            eprintln!("[ROBOT] step {}: spinning for {} steps — backing up",
                                self.state.step, self.spin_steps);
                            self.spin_steps = 0;
                            self.steps_no_progress = 0;
                            self.recovery_phase = RecoveryPhase::Backup { remaining: 12 };
                            cmd = self.recovery_step();
                        }
                    } else {
                        self.spin_steps = 0;
                        if self.best_target_dist2 == f64::MAX || dist2 < self.best_target_dist2 - 0.01 {
                            self.best_target_dist2 = dist2;
                            self.steps_no_progress = 0;
                        } else {
                            self.steps_no_progress += 1;
                            if self.steps_no_progress >= 15 {
                                eprintln!("[ROBOT] step {}: no progress for {} steps (dist={:.2}) – backing up",
                                    self.state.step, self.steps_no_progress, dist2.sqrt());
                                self.recovery_phase = RecoveryPhase::Backup { remaining: 10 };
                                cmd = self.recovery_step();
                            }
                        }
                    }

                    // Cell-revisit detection: if we've been in this cell 4+ times
                    // recently the drone is looping — skip the waypoint and back off.
                    let cell = (self.pose.x as usize, self.pose.y as usize, self.pose.z as usize);
                    self.recent_cells.push_back(cell);
                    if self.recent_cells.len() > 30 {
                        self.recent_cells.pop_front();
                    }
                    let revisits = self.recent_cells.iter().filter(|&&c| c == cell).count();
                    if revisits >= 8 && self.recovery_phase == RecoveryPhase::None {
                        eprintln!("[ROBOT] step {}: cell ({},{},{}) visited {} times recently — skipping waypoint + backing up",
                            self.state.step, cell.0, cell.1, cell.2, revisits);
                        self.recent_cells.clear();
                        if self.path_index + 1 < self.path.len() {
                            self.path_index += 1;
                        }
                        self.recovery_phase = RecoveryPhase::Backup { remaining: 10 };
                        cmd = self.recovery_step();
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

            // Path exhausted but goal not reached — drive directly to the end cell
            // rather than stopping, so a small overshoot doesn't stall the drone.
            let end = self.cave.end;
            let end_node = pathfinding::Node(end.0, end.1, end.2);
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
        if !self.return_initialized {
            self.return_initialized = true;
            self.state.mode = RobotMode::Return;

            let slam_grid = self.slam.map.to_passable_grid();
            let slam_graph = GridGraph::from_passable_grid(&slam_grid);
            let current_cell = (self.pose.x as usize, self.pose.y as usize, self.pose.z as usize);
            let start_node = Node(self.cave.start.0, self.cave.start.1, self.cave.start.2);
            let current_node = Node(current_cell.0, current_cell.1, current_cell.2);

            eprintln!(
                "[ROBOT] Computing return path from ({},{},{}) to ({},{},{}) via SLAM map …",
                current_cell.0, current_cell.1, current_cell.2,
                self.cave.start.0, self.cave.start.1, self.cave.start.2,
            );

            let result = pathfinding::astar(&slam_graph, current_node, start_node);
            self.path = match result {
                Some((p, _)) => {
                    eprintln!("[ROBOT] Return path: {} cells", p.len());
                    p
                }
                None => {
                    eprintln!("[ROBOT] No return path found on SLAM map — stopping");
                    return NavCommand::stop();
                }
            };
            self.path_index = 0;
        }

        let cell = (self.pose.x as usize, self.pose.y as usize, self.pose.z as usize);

        if cell == self.cave.start {
            eprintln!(
                "[ROBOT] Reached START at ({},{},{}) in {} total steps",
                cell.0, cell.1, cell.2, self.state.step
            );
            return NavCommand::stop();
        }

        if self.path_index >= self.path.len() {
            eprintln!("[ROBOT] Return path exhausted — replanning …");
            let slam_grid = self.slam.map.to_passable_grid();
            let slam_graph = GridGraph::from_passable_grid(&slam_grid);
            let start_node = Node(self.cave.start.0, self.cave.start.1, self.cave.start.2);
            let current_node = Node(cell.0, cell.1, cell.2);
            let result = pathfinding::astar(&slam_graph, current_node, start_node);
            self.path = match result {
                Some((p, _)) => p,
                None => {
                    eprintln!("[ROBOT] Return replan failed — stopping");
                    return NavCommand::stop();
                }
            };
            self.path_index = 0;
        }

        let target = self.path[self.path_index];
        let mut cmd = self.move_toward(target, scan);

        let (tx, ty, tz) = shared::gazebo_cell_center((target.0, target.1, target.2));
        let dx = tx - self.pose.x;
        let dy = ty - self.pose.y;
        let dz = tz - self.pose.z;
        let dist2 = dx * dx + dy * dy + dz * dz;
        if dist2 < 0.25 {
            self.path_index += 1;
            self.best_target_dist2 = f64::MAX;
            self.steps_no_progress = 0;
        }

        // Wall avoidance (persistent: stays active for N steps after wall clears)
        if self.recovery_phase != RecoveryPhase::None {
            cmd = self.recovery_step();
        } else if self.wall_too_close(scan) {
            self.avoid_steps_remaining = 5;
            if !self.in_avoidance {
                self.wall_stuck_count += 1;
                if self.wall_stuck_count >= 4 {
                    self.recovery_phase = RecoveryPhase::Backup { remaining: 8 };
                    cmd = self.recovery_step();
                } else {
                    self.in_avoidance = true;
                    cmd = self.avoid_obstacle(scan);
                }
            } else {
                cmd = self.avoid_obstacle(scan);
            }
        } else if self.in_avoidance {
            self.avoid_steps_remaining -= 1;
            if self.avoid_steps_remaining == 0 {
                self.in_avoidance = false;
                self.wall_stuck_count = 0;
            } else {
                cmd = self.avoid_obstacle(scan);
            }
        } else {
            self.wall_stuck_count = 0;
        }

        // Progress check for return phase — same spin + revisit detection as forward.
        if self.recovery_phase == RecoveryPhase::None {
            let is_spinning = cmd.linear_x.abs() < 0.01 && cmd.angular_z.abs() > 0.01;
            if is_spinning {
                self.spin_steps += 1;
                if self.spin_steps >= 20 {
                    eprintln!("[ROBOT] step {} [RETURN] spinning for {} steps — backing up",
                        self.state.step, self.spin_steps);
                    self.spin_steps = 0;
                    self.recovery_phase = RecoveryPhase::Backup { remaining: 12 };
                    cmd = self.recovery_step();
                }
            } else {
                self.spin_steps = 0;
            }
            if self.recovery_phase == RecoveryPhase::None {
            if self.best_target_dist2 == f64::MAX || dist2 < self.best_target_dist2 - 0.01 {
                self.best_target_dist2 = dist2;
                self.steps_no_progress = 0;
            } else {
                self.steps_no_progress += 1;
                if self.steps_no_progress >= 15 {
                    eprintln!("[ROBOT] step {} [RETURN] no progress for {} steps (dist={:.2}) – backing up",
                        self.state.step, self.steps_no_progress, dist2.sqrt());
                    self.recovery_phase = RecoveryPhase::Backup { remaining: 10 };
                    cmd = self.recovery_step();
                }
            }
            } // end recovery_phase == None guard
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
        self.pose.x += cmd.linear_x * self.pose.yaw.cos() * dt;
        self.pose.y += cmd.linear_x * self.pose.yaw.sin() * dt;
        // 1 cave-grid z-unit spans GAZEBO_LEVEL_SPACING_METERS in the world, so
        // a linear_z velocity (m/s) advances the cave-grid z coordinate more slowly.
        self.pose.z += cmd.linear_z * dt;
        self.pose.yaw += cmd.angular_z * dt;
        self.state.pose = self.pose;
    }

    fn discover_obstacles(&mut self, scan: &LidarScan) -> bool {
        let mut discovered = false;
        let n_h = scan.num_horizontal();
        let n_v = scan.num_vertical();

        for vi in 0..n_v {
            let elev = scan.angle_min_vertical + vi as f64 * scan.angle_increment_vertical;
            for hi in 0..n_h {
                let az = scan.angle_min + hi as f64 * scan.angle_increment;
                let measured = scan.range(hi, vi);
                if measured >= scan.range_max - 0.01 {
                    continue;
                }

                let global_az = self.pose.yaw + az;
                let dx = global_az.cos() * elev.cos();
                let dy = global_az.sin() * elev.cos();
                let dz = elev.sin();

                let physical_z = shared::gazebo_physical_z(self.pose.z);
                let hx = (self.pose.x + dx * measured) as usize;
                let hy = (self.pose.y + dy * measured) as usize;
                let hz = ((physical_z + dz * measured) / shared::GAZEBO_LEVEL_SPACING_METERS) as usize;

                if hx < self.cave.size_x && hy < self.cave.size_y && hz < self.cave.size_z {
                    let tile = self.cave.grid[hz][hy][hx];
                    if tile == shared::TILE_WALL && self.graph.is_passable(Node(hx, hy, hz)) {
                        self.graph.set_passable(Node(hx, hy, hz), false);
                        self.planner.update_obstacle(Node(hx, hy, hz), true);
                        discovered = true;
                    }
                }
            }
        }

        discovered
    }

    fn avoid_obstacle(&self, scan: &LidarScan) -> NavCommand {
        let target_body_az = if self.path_index < self.path.len() {
            let t = self.path[self.path_index];
            let (tx, ty, _) = shared::gazebo_cell_center((t.0, t.1, t.2));
            let global = (ty - self.pose.y).atan2(tx - self.pose.x);
            ang_diff(global, self.pose.yaw)
        } else {
            0.0
        };

        let (best_az, best_score) = self.score_azimuths(scan, target_body_az);

        if best_score < 0.3 {
            NavCommand {
                linear_x: -0.3,
                linear_y: 0.0,
                linear_z: 0.0,
                angular_z: if best_az >= 0.0 { 0.5 } else { -0.5 },
            }
        } else if best_az.abs() > 0.3 {
            NavCommand::rotate(if best_az >= 0.0 { 0.7 } else { -0.7 })
        } else {
            NavCommand {
                linear_x: 0.4,
                linear_y: 0.0,
                linear_z: 0.0,
                angular_z: best_az * 2.0,
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
                    self.wall_stuck_count = 0;
                } else {
                    self.recovery_phase = RecoveryPhase::Rotate { remaining: remaining - 1 };
                }
                cmd
            }
            RecoveryPhase::None => NavCommand::stop(),
        }
    }

    fn wall_too_close(&self, scan: &LidarScan) -> bool {
        let n_h = scan.num_horizontal();
        let angle_inc = scan.angle_increment;
        // Cover the full frontal half of the drone (±90°) so arm tips at ±45°
        // are included. min_safe accounts for rotor tips ~0.13 m from centre
        // (0.09 m arm + 0.036 m rotor) plus a ~0.07 m buffer.
        let forward_cone = std::f64::consts::FRAC_PI_2;
        let min_safe = 0.20;

        let elev_limit = 0.35_f64;
        for hi in 0..n_h {
            let az = scan.angle_min + hi as f64 * angle_inc;
            if az.abs() > forward_cone {
                continue;
            }
            let n_v = scan.num_vertical();
            for vi in 0..n_v {
                let elev = scan.angle_min_vertical + vi as f64 * scan.angle_increment_vertical;
                if elev.abs() > elev_limit {
                    continue;
                }
                let measured = scan.range(hi, vi);
                if measured >= scan.range_max - 0.01 {
                    continue;
                }
                if measured < min_safe {
                    return true;
                }
            }
        }
        false
    }

    /// Score each azimuth in the LiDAR scan, preferring clear paths aligned
    /// with the target direction.  Returns (best_azimuth_in_body_frame, score).
    fn score_azimuths(&self, scan: &LidarScan, target_body_az: f64) -> (f64, f64) {
        let n_h = scan.num_horizontal();
        let n_v = scan.num_vertical();
        let angle_inc = scan.angle_increment;
        let mut best_score = -f64::INFINITY;
        let mut best_az = target_body_az;
        let elev_limit = 0.35_f64; // ignore steep up/down rays (floor & ceiling)
        for hi in 0..n_h {
            let az = scan.angle_min + hi as f64 * angle_inc;
            let mut min_at_az = scan.range_max;
            for vi in 0..n_v {
                let elev = scan.angle_min_vertical + vi as f64 * scan.angle_increment_vertical;
                if elev.abs() > elev_limit {
                    continue;
                }
                let r = scan.range(hi, vi);
                if r < min_at_az {
                    min_at_az = r;
                }
            }
            let yaw_diff = ang_diff(target_body_az, az).abs();
            let alignment = (std::f64::consts::PI - yaw_diff) / std::f64::consts::PI;
            let score = min_at_az * (0.3 + 0.7 * alignment);
            if score > best_score {
                best_score = score;
                best_az = az;
            }
        }
        (best_az, best_score)
    }

    /// Proportional heading controller — face the waypoint then drive.
    /// Reactive obstacle avoidance is handled separately by wall_too_close /
    /// avoid_obstacle, so this function just needs to keep the drone aimed
    /// at the next cell centre without any LiDAR scoring that could cause
    /// oscillation or spurious spinning.
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

        if heading_err.abs() > 0.2 {
            // Turn to face the waypoint before driving
            NavCommand {
                linear_x: 0.0,
                linear_y: 0.0,
                linear_z: 0.0,
                angular_z: heading_err.signum() * 0.7,
            }
        } else {
            // Drive forward, steering proportionally to hold the heading
            NavCommand {
                linear_x: 1.0,
                linear_y: 0.0,
                linear_z: 0.0,
                angular_z: heading_err * 2.0,
            }
        }
    }

    /// Pure kinematic waypoint approach — no LiDAR used.
    /// Used during the blind return phase where scan data is ignored.
    /// Drives at half speed to compensate for the lack of obstacle reactions.
    fn move_toward_blind(&self, target: Node) -> NavCommand {
        let (tx, ty, tz) = shared::gazebo_cell_center((target.0, target.1, target.2));
        let dz = tz - self.pose.z;

        if (self.pose.z as usize) != target.2 || dz.abs() > 0.3 {
            return NavCommand {
                linear_x: 0.0,
                linear_y: 0.0,
                linear_z: dz.signum() * 0.5,
                angular_z: 0.0,
            };
        }

        let dx = tx - self.pose.x;
        let dy = ty - self.pose.y;
        let heading_err = ang_diff(dy.atan2(dx), self.pose.yaw);

        if heading_err.abs() > 0.2 {
            NavCommand {
                linear_x: 0.0,
                linear_y: 0.0,
                linear_z: 0.0,
                angular_z: heading_err.signum() * 0.5,
            }
        } else {
            NavCommand {
                linear_x: 0.5,
                linear_y: 0.0,
                linear_z: 0.0,
                angular_z: heading_err * 1.5,
            }
        }
    }
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
