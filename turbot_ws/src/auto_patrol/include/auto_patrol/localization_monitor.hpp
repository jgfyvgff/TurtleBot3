#pragma once

#include <mutex>
#include <optional>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "auto_patrol/patrol_config.hpp"

namespace auto_patrol
{

// 只维护传感器状态，不执行 ROS2 I/O；mutex 保护全部内部状态和稳定性不变量。
class LocalizationMonitor final
{
public:
    struct AmclPose
    {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
    };

    explicit LocalizationMonitor(const PatrolConfig & config);

    // 返回时间戳是否有效，同时记录最近一次 LaserScan。
    bool update_scan(
        const sensor_msgs::msg::LaserScan & message,
        const rclcpp::Time & now);

    // 过期 AMCL 会清零稳定计数，避免使用旧定位启动导航。
    bool update_amcl_pose(
        const geometry_msgs::msg::PoseWithCovarianceStamped & message,
        const rclcpp::Time & now);

    // 全局重定位会使旧粒子分布失效，因此必须丢弃服务调用前的稳定样本。
    void reset_for_relocalization();

    // 扫描匹配阶段只等待 LaserScan，不要求 AMCL 已经发布可用位姿。
    bool scan_is_current(const rclcpp::Time & now) const;
    bool is_ready(const rclcpp::Time & now) const;
    // 返回最近一条通过时间戳校验的 AMCL 位姿，用于与独立扫描匹配结果复核。
    std::optional<AmclPose> latest_amcl_pose() const;

private:
    bool scan_is_current_unlocked(const rclcpp::Time & now) const;
    bool amcl_pose_is_current_unlocked(const rclcpp::Time & now) const;

    const double max_message_age_sec_;
    const double future_message_tolerance_sec_;
    const int stable_amcl_samples_;
    const double stable_position_tolerance_;
    const double stable_yaw_tolerance_;
    const double position_variance_threshold_;
    const double yaw_variance_threshold_;

    mutable std::mutex mutex_;
    bool scan_received_{false};
    bool amcl_pose_received_{false};
    builtin_interfaces::msg::Time last_scan_stamp_;
    builtin_interfaces::msg::Time last_amcl_pose_stamp_;
    std::optional<AmclPose> previous_amcl_pose_;
    std::optional<AmclPose> latest_amcl_pose_;
    int amcl_stable_count_{0};
    bool covariance_received_{false};
    double position_variance_x_{0.0};
    double position_variance_y_{0.0};
    double yaw_variance_{0.0};
};

}  // namespace auto_patrol
