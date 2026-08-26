"""Launch 文件的结构测试，不启动 Gazebo、Nav2 或真实机器人."""

import importlib.util
import os
from pathlib import Path
import unittest

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIRECTORY = PACKAGE_ROOT / "src" / "launch"


def load_launch_module(file_name):
    """从源码目录加载 Launch，避免测试依赖安装路径中的旧版本."""
    module_path = LAUNCH_DIRECTORY / file_name
    spec = importlib.util.spec_from_file_location(module_path.stem, module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load launch file: {module_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class LaunchFileTest(unittest.TestCase):
    """验证两个 Launch 的职责边界和组合入口的保留策略."""

    def test_auto_patrol_launch_starts_only_patrol_node(self):
        launch_module = load_launch_module("auto_patrol.launch.py")
        launch_description = launch_module.generate_launch_description()
        entities = launch_description.entities

        self.assertTrue(
            any(
                isinstance(entity, DeclareLaunchArgument) and
                entity.name == "use_sim_time"
                for entity in entities
            )
        )
        patrol_nodes = [entity for entity in entities if isinstance(entity, Node)]
        self.assertEqual(len(patrol_nodes), 1)
        self.assertEqual(patrol_nodes[0].node_package, "auto_patrol")
        self.assertEqual(patrol_nodes[0].node_executable, "auto_patrol_node")

    def test_combined_launch_includes_world_nav2_and_patrol_node(self):
        previous_model = os.environ.get("TURTLEBOT3_MODEL")
        os.environ["TURTLEBOT3_MODEL"] = "waffle"
        try:
            launch_module = load_launch_module("patrol_with_nav2.launch.py")
            launch_description = launch_module.generate_launch_description()
        finally:
            if previous_model is None:
                del os.environ["TURTLEBOT3_MODEL"]
            else:
                os.environ["TURTLEBOT3_MODEL"] = previous_model

        entities = launch_description.entities
        included_launches = [
            entity
            for entity in entities
            if isinstance(entity, IncludeLaunchDescription)
        ]
        self.assertEqual(
            len(included_launches),
            2,
        )
        self.assertTrue(
            any(
                isinstance(entity, SetEnvironmentVariable)
                for entity in entities
            )
        )
        # 巡检节点退出后保留 Gazebo、Nav2 和 RViz，便于检查最终导航状态。
        self.assertFalse(
            any(
                entity.__class__.__name__ == "RegisterEventHandler"
                for entity in entities
            )
        )
        self.assertTrue(
            any(
                isinstance(entity, Node) and
                entity.node_package == "auto_patrol" and
                entity.node_executable == "auto_patrol_node"
                for entity in entities
            )
        )
