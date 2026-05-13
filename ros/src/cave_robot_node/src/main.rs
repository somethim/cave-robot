use rclrs::*;

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("cave_robot_node")?;

    let _sub = node.create_subscription(
        "/scan",
        |msg: sensor_msgs::msg::LaserScan| {
            let n = msg.ranges.len();
            let min = msg.range_min;
            let max = msg.range_max;
            println!("scan: {n} rays [{min}..{max}]");
        },
    )?;

    let _pub = node.create_publisher::<geometry_msgs::msg::Twist>("/cmd_vel")?;

    eprintln!("cave_robot_node: spinning...");
    executor.spin(SpinOptions::default()).first_error()
}
