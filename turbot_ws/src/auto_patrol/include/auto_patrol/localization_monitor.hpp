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

    // 平移验证开始时只清除 AMCL 证据，保留最近 LaserScan 供前方安全检查使用。
    void begin_translation_validation();

    // 主动探索时允许机器人正常旋转，只要求连续低协方差样本，不比较相邻 yaw。
    bool relocalization_is_ready(const rclcpp::Time & now) const;

    // 仅在当前 LaserScan 的前方扇区存在有效且足够远的量测时允许短距离平移。
    bool front_clearance_is_safe(
        const rclcpp::Time & now,
        double minimum_clearance,
        double sector_half_angle) const;

    bool is_ready(const rclcpp::Time & now) const;

    // 用于停止后的最终确认，必须同时保证 AMCL 位姿仍然新鲜。
    bool confidence_is_sufficient(const rclcpp::Time & now) const;

private:
    struct PoseSample
    {
        double x;
        double y;
        double yaw;
    };

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
    // 拷贝最近一帧扫描供安全判断使用；其访问始终受 mutex_ 保护。
    std::optional<sensor_msgs::msg::LaserScan> latest_scan_;
    std::optional<PoseSample> previous_amcl_pose_;
    int amcl_stable_count_{0};
    int confident_amcl_sample_count_{0};
    bool covariance_received_{false};
    double position_variance_x_{0.0};
    double position_variance_y_{0.0};
    double yaw_variance_{0.0};
};

}  // namespace auto_patrol
