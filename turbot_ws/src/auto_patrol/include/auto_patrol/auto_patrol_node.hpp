#pragma once

#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "auto_patrol/costmap_cleaner.hpp"
#include "auto_patrol/initial_pose_publisher.hpp"
#include "auto_patrol/localization_bootstrapper.hpp"
#include "auto_patrol/localization_monitor.hpp"
#include "auto_patrol/patrol_config.hpp"
#include "auto_patrol/patrol_controller.hpp"
#include "auto_patrol/scan_map_matcher.hpp"

namespace auto_patrol
{

// AutoPatrolNode类负责整个自动巡逻过程的控制和管理
class AutoPatrolNode final : public rclcpp::Node
{
public:
    AutoPatrolNode();//
    ~AutoPatrolNode() override;

    int exit_code() const noexcept;// 返回退出代码

private:
    void handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message);// 处理激光扫描数据
    void handle_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
    void handle_amcl_pose(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message);// 处理AMCL位姿数据
    void startup_tick();
    void start_localization_wait();
    void handle_localization_finished(bool success);
    void start_costmap_cleanup();
    void handle_costmap_cleanup_finished(bool success);
    void handle_patrol_finished(bool success);
    void stop_timers();
    void stop_components();

    PatrolConfig config_;
    std::unique_ptr<LocalizationMonitor> localization_monitor_;// 负责监控定位状态
    std::unique_ptr<ScanMapMatcher> scan_map_matcher_;// 使用静态地图独立校验激光定位结果
    std::unique_ptr<LocalizationBootstrapper> localization_bootstrapper_;// 编排 AMCL 无运动定位状态机
    std::unique_ptr<InitialPosePublisher> initial_pose_publisher_;
    std::unique_ptr<CostmapCleaner> costmap_cleaner_;
    std::unique_ptr<PatrolController> patrol_controller_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        amcl_pose_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
        map_subscription_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    rclcpp::TimerBase::SharedPtr localization_timer_;

    bool startup_finished_{false};
    bool finished_{false};
    int exit_code_{0};
};

}  // namespace auto_patrol
