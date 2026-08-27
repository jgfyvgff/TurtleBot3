#include "auto_patrol/localization_monitor.hpp"

#include <cmath>

#include "auto_patrol/pose_utils.hpp"

namespace auto_patrol
{

LocalizationMonitor::LocalizationMonitor(const PatrolConfig & config)
    : max_message_age_sec_(config.max_message_age_sec),
      future_message_tolerance_sec_(config.future_message_tolerance_sec),
      stable_amcl_samples_(config.stable_amcl_samples),
      stable_position_tolerance_(config.stable_position_tolerance),
      stable_yaw_tolerance_(config.stable_yaw_tolerance),
      position_variance_threshold_(
          config.localization_position_variance_threshold),
      yaw_variance_threshold_(config.localization_yaw_variance_threshold)
{
}

void LocalizationMonitor::reset_for_relocalization()
{
    std::lock_guard<std::mutex> lock(mutex_);
    // 要求重定位请求之后重新收到扫描和 AMCL 位姿，避免复用旧粒子状态。
    scan_received_ = false;
    amcl_pose_received_ = false;
    previous_amcl_pose_.reset();
    latest_amcl_pose_.reset();
    amcl_stable_count_ = 0;
    covariance_received_ = false;
}

bool LocalizationMonitor::update_scan(
    const sensor_msgs::msg::LaserScan & message,
    const rclcpp::Time & now)
{
    std::lock_guard<std::mutex> lock(mutex_);
    scan_received_ = true;//锁内设置scan_received_为true，表示已经收到扫描数据
    last_scan_stamp_ = message.header.stamp;
    return pose_utils::timestamp_is_current(
        message.header.stamp,
        now,
        max_message_age_sec_,
        future_message_tolerance_sec_);
}

bool LocalizationMonitor::update_amcl_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & message,
    const rclcpp::Time & now)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pose_utils::timestamp_is_current(
            message.header.stamp,
            now,
            max_message_age_sec_,
            future_message_tolerance_sec_))
    {
        amcl_stable_count_ = 0;
        previous_amcl_pose_.reset();
        covariance_received_ = false;
        return false;
    }

    const auto & pose = message.pose.pose;
    const double position_variance_x = message.pose.covariance[0];
    const double position_variance_y = message.pose.covariance[7];
    const double yaw_variance = message.pose.covariance[35];
    const AmclPose current_pose{
        pose.position.x,
        pose.position.y,
        pose_utils::quaternion_to_yaw(pose.orientation)};

    if (previous_amcl_pose_.has_value()) {
        const double position_difference = std::hypot(
            current_pose.x - previous_amcl_pose_->x,
            current_pose.y - previous_amcl_pose_->y);
        const double yaw_difference = std::abs(pose_utils::angle_difference(
            current_pose.yaw,
            previous_amcl_pose_->yaw));

        if (position_difference <= stable_position_tolerance_ &&
            yaw_difference <= stable_yaw_tolerance_)
        {
            amcl_stable_count_ = std::min(
                amcl_stable_count_ + 1,
                stable_amcl_samples_);
        } else {
            amcl_stable_count_ = 1;
        }
    } else {
        amcl_stable_count_ = 1;
    }

    previous_amcl_pose_ = current_pose;
    latest_amcl_pose_ = current_pose;
    amcl_pose_received_ = true;
    last_amcl_pose_stamp_ = message.header.stamp;
    const bool covariance_is_good =
        std::isfinite(position_variance_x) &&
        std::isfinite(position_variance_y) &&
        std::isfinite(yaw_variance) &&
        position_variance_x >= 0.0 &&
        position_variance_y >= 0.0 &&
        yaw_variance >= 0.0 &&
        position_variance_x <= position_variance_threshold_ &&
        position_variance_y <= position_variance_threshold_ &&
        yaw_variance <= yaw_variance_threshold_;
    covariance_received_ = covariance_is_good;
    position_variance_x_ = position_variance_x;
    position_variance_y_ = position_variance_y;
    yaw_variance_ = yaw_variance;
    if (!covariance_is_good) {
        // 稳定窗口中的低置信度样本会使整段窗口失效，避免只凭最新一帧放行。
        amcl_stable_count_ = 0;
    }
    return true;
}

bool LocalizationMonitor::scan_is_current_unlocked(const rclcpp::Time & now) const
{
    return scan_received_ && pose_utils::timestamp_is_current(
        last_scan_stamp_,
        now,
        max_message_age_sec_,
        future_message_tolerance_sec_);
}

bool LocalizationMonitor::amcl_pose_is_current_unlocked(
    const rclcpp::Time & now) const
{
    return amcl_pose_received_ && pose_utils::timestamp_is_current(
        last_amcl_pose_stamp_,
        now,
        max_message_age_sec_,
        future_message_tolerance_sec_);
}

bool LocalizationMonitor::is_ready(
    const rclcpp::Time & now) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!scan_is_current_unlocked(now) ||
        !amcl_pose_is_current_unlocked(now))
    {
        return false;
    }
    return amcl_pose_received_ &&
        amcl_stable_count_ >= stable_amcl_samples_ &&
        covariance_received_ &&
        position_variance_x_ >= 0.0 &&
        position_variance_y_ >= 0.0 &&
        yaw_variance_ >= 0.0 &&
        position_variance_x_ <= position_variance_threshold_ &&
        position_variance_y_ <= position_variance_threshold_ &&
        yaw_variance_ <= yaw_variance_threshold_;
}

bool LocalizationMonitor::scan_is_current(const rclcpp::Time & now) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return scan_is_current_unlocked(now);
}

std::optional<LocalizationMonitor::AmclPose>
LocalizationMonitor::latest_amcl_pose() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_amcl_pose_;
}

}  // namespace auto_patrol
