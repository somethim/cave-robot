#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

if ! command -v colcon &> /dev/null; then
    echo "ERROR: colcon not found. Install inside the distrobox:"
    echo "  pip install colcon-cargo colcon-ros-cargo"
    exit 1
fi

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "ERROR: ROS 2 Jazzy not found at /opt/ros/jazzy. Run inside the distrobox."
    exit 1
fi

if [ ! -d deps/rosidl_rust ]; then
    git submodule update --init --recursive
fi

. /opt/ros/jazzy/setup.bash
colcon build --symlink-install --paths src deps

echo ""
echo "Done. To run:"
echo "  source ros/install/setup.bash"
echo "  ros2 run cave_robot_node cave_robot_node"
