"""一次启动 TurtleBot3 世界、Nav2 与自动巡检节点的 Launch 入口."""

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
    """让世界、Nav2 和巡检节点共用 waffle 模型与 Gazebo 仿真时间."""
    auto_patrol_share = get_package_share_directory("auto_patrol")
    turtlebot3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
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
    world_launch = os.path.join(
        turtlebot3_gazebo_share,
        "launch",
        "turtlebot3_world.launch.py",
    )
    default_map = os.path.join(turtlebot3_navigation_share, "map", "map.yaml")
    map_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")

    # 巡检节点内部会等待 Action Server、激光和 AMCL 数据，
    # 因而无需用固定延迟猜测 Gazebo、Nav2 的实际启动完成时机。
    patrol_node = Node(
        package="auto_patrol",
        executable="auto_patrol_node",
        name="auto_patrol",
        parameters=[patrol_parameters, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Whether to use the Gazebo simulation clock.",
        ),
        DeclareLaunchArgument(
            "x_pose",
            default_value="-2.0",
            description="Initial TurtleBot3 x coordinate in turtlebot3_world.",
        ),
        DeclareLaunchArgument(
            "y_pose",
            default_value="-0.5",
            description="Initial TurtleBot3 y coordinate in turtlebot3_world.",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=default_map,
            description="Full path to the Nav2 map YAML file.",
        ),
        # 官方 Gazebo 与 Nav2 Launch 都在载入时读取该环境变量；作业统一使用 waffle。
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "waffle"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(world_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "x_pose": x_pose,
                "y_pose": y_pose,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                "map": map_file,
                "params_file": nav2_parameters,
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        patrol_node,
    ])
