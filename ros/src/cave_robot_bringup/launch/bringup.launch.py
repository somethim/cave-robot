from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    gz_launch = os.path.join(
        get_package_share_directory("cave_robot_gazebo"),
        "launch",
        "gazebo.launch.py",
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
        ),

        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                "/cave_robot/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                "/model/cave_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            ],
            output="screen",
        ),

        ExecuteProcess(
            cmd=["ros2", "run", "cave_robot_node", "cave_robot_node"],
            output="screen",
        ),
    ])
