build:
    cargo build --workspace --release

start:
    ./target/release/generator
    ./target/release/robot

check:
    cargo check --workspace
    cargo clippy --workspace -- -D warnings

test:
    cargo test --workspace

generate:
    cargo run -p generator --bin generator

generate-world:
    cargo run -p generator --bin cave_to_world

robot:
    cargo run -p robot

dev:
    cargo run -p generator --bin generator
    cargo run -p robot

ros-bootstrap:
    ./ros/scripts/ros-bootstrap

ros-build:
    ./ros/scripts/ros-build

ros-robot:
    ./ros/scripts/ros-robot

ros-clean:
    rm -rf ros/build ros/install ros/log ros/.cargo

ros-rebuild:
    just ros-clean
    just ros-bootstrap

sim:
    ./ros/scripts/ros-run ros2 launch cave_robot_gazebo gazebo.launch.py

sim-dev:
    just dev
    just generate-world
    just ros-build
    just sim

bringup:
    ./ros/scripts/ros-run ros2 launch cave_robot_bringup bringup.launch.py
