"""启动 CARLA 桥 +（可选）最小控制节点。

用法:
    source setup/activate.sh
    cd ros2_ws && colcon build --symlink-install
    ros2 launch carla_autonomy bridge.launch.py
    ros2 launch carla_autonomy bridge.launch.py weather:=clear autopilot:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("weather", default_value="rain_night"),
        DeclareLaunchArgument("town", default_value=""),
        DeclareLaunchArgument("autopilot", default_value="false"),
        DeclareLaunchArgument("spawn_ego", default_value="true"),
        DeclareLaunchArgument("sync_mode", default_value="true"),
        DeclareLaunchArgument("rate_hz", default_value="20.0"),
        DeclareLaunchArgument("camera_width", default_value="1280"),
        DeclareLaunchArgument("camera_height", default_value="720"),
        DeclareLaunchArgument("lidar_channels", default_value="32"),
        DeclareLaunchArgument("with_control", default_value="false"),
        DeclareLaunchArgument("control_pattern", default_value="straight"),
    ]

    bridge = Node(
        package="carla_autonomy",
        executable="carla_bridge",
        name="carla_bridge",
        output="screen",
        parameters=[{
            "weather": LaunchConfiguration("weather"),
            "town": LaunchConfiguration("town"),
            "autopilot": LaunchConfiguration("autopilot"),
            "spawn_ego": LaunchConfiguration("spawn_ego"),
            "sync_mode": LaunchConfiguration("sync_mode"),
            "rate_hz": LaunchConfiguration("rate_hz"),
            "camera_width": LaunchConfiguration("camera_width"),
            "camera_height": LaunchConfiguration("camera_height"),
            "lidar_channels": LaunchConfiguration("lidar_channels"),
        }],
    )

    control = Node(
        package="carla_autonomy",
        executable="carla_control",
        name="carla_control",
        output="screen",
        condition=IfCondition(LaunchConfiguration("with_control")),
        parameters=[{"pattern": LaunchConfiguration("control_pattern")}],
    )

    return LaunchDescription(args + [bridge, control])

