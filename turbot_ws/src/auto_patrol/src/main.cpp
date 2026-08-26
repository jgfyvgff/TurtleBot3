#include "auto_patrol/auto_patrol_node.hpp"

#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

// 进程入口只负责 ROS2 初始化、事件循环和退出码处理，节点内部保留业务编排职责。
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<auto_patrol::AutoPatrolNode>();
        rclcpp::spin(node);

        const int exit_code = node->exit_code();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        return exit_code;
    } catch (const std::exception & exception) {
        RCLCPP_ERROR(
            rclcpp::get_logger("auto_patrol"),
            "Failed to start auto_patrol: %s",
            exception.what());
        rclcpp::shutdown();
        return 1;
    }
}
