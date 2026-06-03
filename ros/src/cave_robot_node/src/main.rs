use rclrs::*;
use std::io::{BufRead, BufReader, Write};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;

use shared::robot::LidarScan;

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("cave_robot_node")?;

    let robot_path = std::env::var("CAVE_ROBOT_BIN")
        .unwrap_or_else(|_| "./target/release/robot".to_string());
    let cave_file = std::env::var("CAVE_FILE")
        .unwrap_or_else(|_| "./tmp/cave.json".to_string());

    let _cave_json = std::fs::read_to_string(&cave_file)
        .expect("failed to read cave file");

    let mut child = Command::new(&robot_path)
        .args(["--pipe"])
        .env("CAVE_FILE", &cave_file)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("failed to spawn robot binary");

    let child_stdin = child.stdin.take().expect("failed to open robot stdin");
    let child_stdout = child.stdout.take().expect("failed to open robot stdout");

    let stdin_mutex = Arc::new(Mutex::new(child_stdin));
    let robot_done = Arc::new(AtomicBool::new(false));

    // Gazebo LiDAR → LidarScan JSON → robot stdin
    // Pose is NOT embedded because the Gazebo PosePublisher reports the model
    // at (0,0,0) instead of the world <include><pose>.  The robot's built-in
    // dead-reckoning (proven by --self-scan) handles navigation correctly.
    let lidar_stdin = Arc::clone(&stdin_mutex);
    let lidar_done = Arc::clone(&robot_done);
    let _lidar_sub = node.create_subscription::<sensor_msgs::msg::LaserScan, (sensor_msgs::msg::LaserScan,)>(
        "/cave_robot/lidar",
        move |msg: sensor_msgs::msg::LaserScan| {
            if lidar_done.load(Ordering::Relaxed) {
                return;
            }
            let sanitise = |v: f32| { let r = v as f64; if r.is_finite() { r } else { 0.0 } };
            let ranges: Vec<f64> = msg.ranges.iter().map(|&r| sanitise(r).max(0.0)).collect();
            let angle_increment = sanitise(msg.angle_increment);
            if angle_increment == 0.0 {
                return;
            }
            let scan = LidarScan {
                ranges,
                angle_min: sanitise(msg.angle_min),
                angle_max: sanitise(msg.angle_max),
                angle_increment,
                angle_min_vertical: 0.0,
                angle_max_vertical: 0.0,
                angle_increment_vertical: 0.0,
                range_min: sanitise(msg.range_min).max(0.01),
                range_max: sanitise(msg.range_max).max(1.0),
                pose: None,
            };
            let json = serde_json::to_string(&scan).expect("[NODE] serialise LidarScan");
            let mut stdin = lidar_stdin.lock().unwrap();
            if let Err(e) = writeln!(stdin, "{}", json) {
                // Robot process exited — stop feeding it scans.
                lidar_done.store(true, Ordering::Relaxed);
                eprintln!("[NODE] robot pipe closed ({e}), stopping scan feed");
            }
        },
    )?;

    let pub_cmd =
        node.create_publisher::<geometry_msgs::msg::Twist>("/model/cave_robot/cmd_vel")?;

    // Read velocity commands from robot stdout → publish to Gazebo
    let stdout_done = Arc::clone(&robot_done);
    let _stdout_thread = thread::spawn(move || {
        let reader = BufReader::new(child_stdout);
        for line in reader.lines() {
            match line {
                Ok(json_str) => {
                    if let Ok(cmd) = serde_json::from_str::<serde_json::Value>(&json_str) {
                        let lx = cmd["linear_x"].as_f64().unwrap_or(0.0);
                        let lz = cmd["linear_z"].as_f64().unwrap_or(0.0);
                        let az = cmd["angular_z"].as_f64().unwrap_or(0.0);
                        eprintln!("[NODE] cmd: lx={lx:.2} lz={lz:.2} az={az:.2}");
                        let mut twist = geometry_msgs::msg::Twist::default();
                        twist.linear.x = lx;
                        twist.linear.z = lz * shared::GAZEBO_LEVEL_SPACING_METERS;
                        twist.angular.z = az;
                        let _ = pub_cmd.publish(twist);
                    }
                }
                Err(e) => {
                    eprintln!("[NODE] robot stdout error: {e}");
                    break;
                }
            }
        }
        stdout_done.store(true, Ordering::Relaxed);
        eprintln!("[NODE] robot finished — stdout closed");
    });

    eprintln!("[NODE] cave_robot_node: spinning (lidar→robot, robot→cmd_vel)…");
    let result = executor.spin(SpinOptions::default()).first_error();

    let _ = child.kill();
    let _ = child.wait();

    result
}
