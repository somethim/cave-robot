build:
    cargo build --workspace

check:
    cargo check --workspace

test:
    cargo test --workspace

generate:
    cargo run -p generator

robot:
    cargo run -p robot

dev:
	cargo run -p generator
	cargo run -p robot

ros-build:
    cd ros && colcon build --symlink-install

ros-clean:
    rm -rf ros/build ros/install ros/log

ros-rebuild:
    just ros-clean
    just ros-build

ros-source:
    @echo "Run this manually:"
    @echo "source ros/install/setup.bash"

sim:
    ./ros/scripts/ros-run ros2 launch cave_robot_gazebo gazebo.launch.py

bringup:
    ./ros/scripts/ros-run ros2 launch cave_robot_bringup bringup.launch.py
