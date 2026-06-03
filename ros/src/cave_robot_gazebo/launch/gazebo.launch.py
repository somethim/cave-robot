import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def _package_paths(package_name: str) -> tuple[Path, Path]:
    package_prefix = Path(get_package_prefix(package_name)).resolve()
    package_share = Path(get_package_share_directory(package_name)).resolve()
    return package_prefix, package_share


def _default_world_path() -> Path:
    env_world = os.environ.get("CAVE_WORLD_FILE")
    if env_world:
        return Path(env_world).expanduser().resolve()

    gazebo_prefix, gazebo_share = _package_paths("cave_robot_gazebo")
    source_world = (
        gazebo_prefix.parent.parent / "src" / "cave_robot_gazebo" / "worlds" / "generated" / "cave.world"
    )
    if source_world.exists():
        return source_world

    return gazebo_share / "worlds" / "generated" / "cave.world"


def _resource_path() -> str:
    _, gazebo_share = _package_paths("cave_robot_gazebo")
    _, description_share = _package_paths("cave_robot_description")
    extra_paths = [
        gazebo_share / "worlds",
        description_share / "urdf",
    ]
    current = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    return os.pathsep.join([str(path) for path in extra_paths] + ([current] if current else []))


def _gz_command(context) -> list[str]:
    command = ["gz", "sim"]
    if LaunchConfiguration("headless").perform(context).lower() == "true":
        command.extend(["-s", "--headless-rendering"])
    command.extend(["-r", LaunchConfiguration("world")])
    return command


def _launch_gz(context):
    return [
        ExecuteProcess(
            cmd=_gz_command(context),
            output="screen",
        )
    ]


def generate_launch_description():
    world_path = _default_world_path()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=str(world_path),
                description="Path to the Gazebo world file to load",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo without the GUI window",
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", _resource_path()),
            SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
            SetEnvironmentVariable("QT_STYLE_OVERRIDE", ""),
            SetEnvironmentVariable("GDK_BACKEND", "x11"),
            SetEnvironmentVariable("SDL_VIDEODRIVER", "x11"),
            SetEnvironmentVariable("QT_X11_NO_MITSHM", "1"),
            SetEnvironmentVariable("WAYLAND_DISPLAY", ""),
            SetEnvironmentVariable("QT_OPENGL", os.environ.get("QT_OPENGL", "desktop")),
            LogInfo(msg=["Launching Gazebo with world: ", LaunchConfiguration("world")]),
            LogInfo(msg=["Headless mode: ", LaunchConfiguration("headless")], condition=IfCondition(LaunchConfiguration("headless"))),
            LogInfo(msg=["GUI mode enabled"], condition=UnlessCondition(LaunchConfiguration("headless"))),
            LogInfo(msg=["QT_OPENGL=", os.environ.get("QT_OPENGL", "desktop")]),
            OpaqueFunction(function=_launch_gz),
        ]
    )
