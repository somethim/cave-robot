# Setup

The project runs inside a **Distrobox** container with ROS 2 Jazzy and Rust.
The host machine only needs Distrobox; everything else installs inside the
container.

## Prerequisites

### Create the distrobox

```bash
distrobox create --image ubuntu:24.04 --name cave-robot-ros-two
distrobox enter cave-robot-ros-two
```

### Install ROS 2 Jazzy

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-test-msgs \
  ros-jazzy-test-interface-files \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  python3-colcon-common-extensions \
  mesa-utils
```

### Install Rust

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env
```

### Install build tools

```bash
sudo apt install -y build-essential cmake git just
pip install colcon-cargo colcon-ros-cargo catkin_pkg lark
```

### Shell configuration

Add to `~/.bashrc` inside the distrobox so ROS is sourced automatically:

```bash
if [ -n "$DISTROBOX_ENTER_PATH" ] || [ -f /.dockerenv ]; then
  if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
  fi
fi
```

Reload: `source ~/.bashrc`

## Clone and build

```bash
git clone --recursive <repo-url> ~/Projects/cave-robot
cd ~/Projects/cave-robot

# Non-ROS workspace (generator, offline robot, crates)
cargo build --release --workspace

# ROS 2 workspace — external message packages + cave_robot_node
just ros-bootstrap
```

`ros-bootstrap` initialises submodules, builds external message packages, and
builds the project's ROS packages. Only needed once or after adding new message
dependencies.

For incremental builds after code changes:

```bash
cargo build --release --workspace   # non-ROS crates
just ros-build                      # cave_robot_node only
```

## Usage

### Full stack (recommended)

```bash
just all
```

Runs: `build` → `generate` → `generate-world` → `bringup`

### Step by step

```bash
just generate        # Generate cave.json (random or seeded)
just generate-world  # Convert cave.json → Gazebo SDF world
just bringup         # Launch Gazebo + ROS bridge + robot node
```

### Offline mode (no Gazebo)

```bash
just generate
just robot           # runs robot binary with simulated LiDAR
```

Or with a custom cave:

```bash
CAVE_FILE=./tmp/cave.json ./target/release/robot
```

### Gazebo only (visual inspection)

```bash
just generate
just generate-world
just sim
```

### Environment variables

| Variable | Default | Meaning |
|---|---|---|
| `CAVE_FILE` | `cave.json` | Path to cave JSON |
| `CAVE_WORLD_FILE` | `ros/src/.../generated/cave.world` | Gazebo world output |
| `CAVE_ROBOT_SDF_FILE` | `ros/src/.../urdf/cave_robot.sdf` | Drone SDF model |
| `CAVE_SEED` | random | Generator seed (integer) |
| `CAVE_SIZE_X` | `64` | Cave width (cells) |
| `CAVE_SIZE_Y` | `32` | Cave depth (cells) |
| `CAVE_SIZE_Z` | `4` | Cave levels |
| `CAVE_ROBOT_BIN` | `./target/release/robot` | Robot binary path (node) |

## justfile commands

| Command | What |
|---|---|
| `just all` | build + generate + generate-world + bringup |
| `just dev` | build + generate + generate-world + sim |
| `just build` | cargo build --release + ros-build |
| `just generate` | Run generator → cave.json |
| `just generate-world` | Run cave_to_world → cave.world |
| `just robot` | Run offline robot binary |
| `just ros-bootstrap` | Init submodules + full colcon build |
| `just ros-build` | Incremental colcon build (project packages only) |
| `just ros-robot` | Run ROS robot node directly |
| `just sim` | Launch Gazebo (GUI) with generated world |
| `just bringup` | Full: Gazebo + bridge + robot node |
| `just check` | cargo check + clippy |
| `just test` | cargo test |

## Known issues

### `test_msgs` required at link time

`rclrs` discovers `test_msgs` from the system and generates link directives.
Install the apt packages:

```bash
sudo apt install ros-jazzy-test-msgs ros-jazzy-test-interface-files
```

### `mise` Python overrides system Python

ROS 2 Jazzy expects system Python 3.12. If `mise` sets Python 3.14, install
colcon plugins for the same Python that runs `colcon`:

```bash
pip install colcon-cargo colcon-ros-cargo catkin_pkg lark
```

### Distrobox shares home — pip packages reach the host

`pip install` inside the distrobox also writes to the host. To isolate:

```bash
python3 -m venv ~/.ros-venv
# add `source ~/.ros-venv/bin/activate` to the distrobox block in ~/.bashrc
pip install colcon-cargo colcon-ros-cargo catkin_pkg lark
```

### Message crates yanked from crates.io

`sensor_msgs`, `geometry_msgs`, etc. are yanked on crates.io. They are built
locally via `rosidl_rust` and patched into cargo resolution via
`.cargo/config.toml` generated by colcon. This is handled automatically by
`ros-bootstrap`.

### `--paths src` silent failure (colcon ≥ 0.20.1)

Passing a directory to `--paths` may silently skip all packages. Use
`--paths src/*` or `--packages-select <name>`. The `ros-build` script handles
this.

### `just` uses `sh`, ROS scripts need `bash`

All ROS commands are wrapped in `ros/scripts/*` bash scripts to avoid shell
compatibility issues.
