#pragma once

#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "auto_patrol/costmap_cleaner.hpp"
#include "auto_patrol/initial_pose_publisher.hpp"
#include "auto_patrol/localization_bootstrapper.hpp"
#include "auto_patrol/localization_monitor.hpp"
#include "auto_patrol/patrol_config.hpp"
#include "auto_patrol/patrol_controller.hpp"

namespace auto_patrol
{

// ROS2 Node 只组装组件、接收消息、驱动 Timer，并将业务终态转换为进程退出码。
class AutoPatrolNode final : public rclcpp::Node
{
public:
    AutoPatrolNode();
    ~AutoPatrolNode() override;

    int exit_code() const noexcept;

private:
    void handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message);
    void handle_odometry(const nav_msgs::msg::Odometry::SharedPtr message);
    void handle_amcl_pose(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message);
    void startup_tick();
    void start_localization_wait();
    void handle_localization_finished(bool success);
    void start_costmap_cleanup();
    void handle_costmap_cleanup_finished(bool success);
    void handle_patrol_finished(bool success);
    void stop_timers();
    void stop_components();

    PatrolConfig config_;
    std::unique_ptr<LocalizationMonitor> localization_monitor_;
    std::unique_ptr<LocalizationBootstrapper> localization_bootstrapper_;
    std::unique_ptr<InitialPosePublisher> initial_pose_publisher_;
    std::unique_ptr<CostmapCleaner> costmap_cleaner_;
    std::unique_ptr<PatrolController> patrol_controller_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        amcl_pose_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odometry_subscription_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    rclcpp::TimerBase::SharedPtr localization_timer_;

    bool startup_finished_{false};
    bool finished_{false};
    int exit_code_{0};
};

}  // namespace auto_patrol
