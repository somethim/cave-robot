all:
    just build
    just generate
    just generate-world
    just bringup

dev:
    just build
    just generate
    just generate-world
    just sim

build:
    cargo build --release --workspace
    just ros-build

sim:
    ./ros/scripts/ros-run ros2 launch cave_robot_gazebo gazebo.launch.py

start:
    ./target/release/generator
    ./target/release/robot

check:
    cargo check --workspace
    cargo clippy --workspace -- -D warnings

test:
    cargo test --workspace

generate:
    cargo build --release --workspace
    ./target/release/generator

generate-world:
    cargo build --release --workspace
    ./target/release/cave_to_world

robot:
    cargo build --release --workspace
    ./target/release/robot

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

bringup:
    ./ros/scripts/ros-run ros2 launch cave_robot_bringup bringup.launch.py
