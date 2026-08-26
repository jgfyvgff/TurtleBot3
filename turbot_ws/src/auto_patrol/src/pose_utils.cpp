#include "auto_patrol/pose_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace auto_patrol::pose_utils
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

double normalize_angle(double angle)
{
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

double angle_difference(double first, double second)
{
    return normalize_angle(first - second);
}

double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & quaternion)
{
    const double sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y);
    const double cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z);
    return std::atan2(sin_yaw, cos_yaw);
}

geometry_msgs::msg::Pose make_pose(double x, double y, double yaw)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(yaw * 0.5);
    pose.orientation.w = std::cos(yaw * 0.5);
    return pose;
}

builtin_interfaces::msg::Time to_message_time(const rclcpp::Time & time)
{
    constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000;
    const std::int64_t total_nanoseconds = time.nanoseconds();
    std::int64_t seconds = total_nanoseconds / kNanosecondsPerSecond;
    std::int64_t nanoseconds = total_nanoseconds % kNanosecondsPerSecond;

    // ROS2 时间消息要求 nanosec 位于 [0, 1e9)，因此负余数需要借位修正。
    if (nanoseconds < 0) {
        --seconds;
        nanoseconds += kNanosecondsPerSecond;
    }

    builtin_interfaces::msg::Time message;
    message.sec = static_cast<std::int32_t>(seconds);
    message.nanosec = static_cast<std::uint32_t>(nanoseconds);
    return message;
}

std::chrono::milliseconds milliseconds_from_seconds(double seconds)
{
    const auto milliseconds = static_cast<std::int64_t>(
        std::llround(seconds * 1000.0));
    return std::chrono::milliseconds(std::max<std::int64_t>(1, milliseconds));
}

bool timestamp_is_current(
    const builtin_interfaces::msg::Time & stamp,
    const rclcpp::Time & now,
    double max_age_sec,
    double future_tolerance_sec)
{
    const double stamp_seconds = rclcpp::Time(stamp).seconds();
    const double now_seconds = now.seconds();
    if (stamp_seconds <= 0.0 || now_seconds <= 0.0) {
        return false;
    }

    const double age = now_seconds - stamp_seconds;
    return age <= max_age_sec && age >= -future_tolerance_sec;
}

}  // namespace auto_patrol::pose_utils
