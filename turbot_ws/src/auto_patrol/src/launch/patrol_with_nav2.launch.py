"""启动 TurtleBot3 Nav2 与自动巡检节点的 Launch 入口."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """让 Nav2 和 auto_patrol 共用本包内的配置及 Gazebo 仿真时间."""
    auto_patrol_share = get_package_share_directory("auto_patrol")
    turtlebot3_navigation_share = get_package_share_directory(
        "turtlebot3_navigation2"
    )
    patrol_parameters = os.path.join(auto_patrol_share, "config", "patrol.yaml")
    nav2_parameters = os.path.join(
        auto_patrol_share,
        "config",
        "nav2_waffle.yaml",
    )
    navigation_launch = os.path.join(
        turtlebot3_navigation_share,
        "launch",
        "navigation2.launch.py",
    )
    default_map = os.path.join(turtlebot3_navigation_share, "map", "map.yaml")
    map_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Whether to use the Gazebo simulation clock.",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=default_map,
            description="Full path to the Nav2 map YAML file.",
        ),
        # 官方 Launch 在载入时读取该环境变量；作业统一使用 waffle。
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "waffle"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                "map": map_file,
                "params_file": nav2_parameters,
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        # 节点会等待 NavigateToPose Action Server，因此无需人为安排启动延迟。
        Node(
            package="auto_patrol",
            executable="auto_patrol_node",
            name="auto_patrol",
            parameters=[patrol_parameters, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
    ])
