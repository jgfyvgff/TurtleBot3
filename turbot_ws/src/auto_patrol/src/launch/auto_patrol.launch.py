"""仅启动自动巡检节点的 Launch 入口."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """加载本包巡检参数；Nav2 已由外部终端或 Launch 启动."""
    package_share = get_package_share_directory("auto_patrol")
    patrol_parameters = os.path.join(package_share, "config", "patrol.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        # 仿真默认启用该选项，避免 auto_patrol 与 Nav2 使用不同时间基准。
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Whether to use the Gazebo simulation clock.",
        ),
        Node(
            package="auto_patrol",
            executable="auto_patrol_node",
            name="auto_patrol",
            parameters=[patrol_parameters, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
    ])
