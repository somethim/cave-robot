# Cave Robot

An autonomous drone that navigates randomly generated cave systems.

## Overview

The drone is placed inside a procedurally generated cave with a randomly selected start and end point. Using a LIDAR
system, it navigates from start to end. Once it reaches the destination, the LIDAR is cut off and the drone must
navigate back to the start using only its stored map via a SLAM algorithm.

### Cave Generation

Caves are generated using a **Seeded Cellular Automata with Region Connection** algorithm. Given a seed, the algorithm
produces a unique cave layout with connected passageways, guaranteeing a valid path between the start and end points.

### Navigation

- **Forward (Start → End):** Real-time LIDAR-based navigation — the drone scans its surroundings and plans its path
  through unexplored cave sections.
- **Return (End → Start):** SLAM-based navigation — the drone relies on the map it built during the forward journey,
  with no active LIDAR input.

## Project Structure

```
├── apps/
│   ├── generator/        # Cave map generator (Rust)
│   └── robot/            # Robot control loop (Rust)
├── crates/
│   ├── shared/           # Shared types: Pose, CaveMap, RobotState, etc.
│   └── pathfinding/      # A*, BFS, and other pathfinding algorithms
├── ros/
│   ├── scripts/
│   │   └── ros-run                # Helper to source ROS setup then run a command
│   └── src/
│       ├── cave_robot_bringup/    # ROS 2 launch files
│       ├── cave_robot_description/ # URDF / robot model
│       ├── cave_robot_gazebo/     # Gazebo simulation worlds
│       └── cave_robot_msgs/       # Custom ROS 2 messages
├── Cargo.toml            # Rust workspace root
├── Justfile              # Convenience commands
└── distrobox-installed-pkgs  # Reference list of distrobox packages
```

## Prerequisites

This project runs inside a **Distrobox** container.

### 1. Create the Distrobox

```bash
distrobox create --image ubuntu:24.04 --name cave-robot-ros-two
distrobox enter cave-robot-ros-two
```

### 2. Install ROS 2 Jazzy

```bash
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions
```

### 3. Install Rust

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env
```

### 4. Install Development Tools

```bash
sudo apt install -y build-essential cmake git just vim
```

### 5. Install Python Dependencies

Some ROS 2 build tools require `catkin_pkg`. If you use a Python version manager (mise, pyenv, etc.), install it via
pip:

```bash
pip install catkin_pkg
```

## Shell Configuration

Add the following to your `~/.bashrc` (or `~/.zshrc`) inside the distrobox:

```bash
if [ -n "$DISTROBOX_ENTER_PATH" ] || [ -f /.dockerenv ]; then
  if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
  fi

  if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
  fi
fi
```

Then reload:

```bash
source ~/.bashrc
```

## Build

```bash
# Clone the repository
git clone <repo-url> ~/cave-robot
cd ~/cave-robot

# Build Rust workspace
cargo build --workspace

# Build ROS 2 workspace
cd ros
colcon build --symlink-install
cd ..
```

## Usage

```bash
# Generate a cave
just generate

# Run the robot
just robot

# Or both in sequence
just dev

# Launch the Gazebo simulation
just sim

# Launch the full robot system
just bringup
```

See the `Justfile` for all available commands.

# Resources used

- Cave generation: [Cellular Automata Method for Generating Random Cave-Like Levels
  ](https://www.roguebasin.com/index.php/Cellular_Automata_Method_for_Generating_Random_Cave-Like_Levels)
  by [RougeBasin](https://roguebasin.com/)