#pragma once

#include <chrono>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/time.hpp"

namespace auto_patrol::pose_utils
{

double normalize_angle(double angle);

double angle_difference(double first, double second);

double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & quaternion);

geometry_msgs::msg::Pose make_pose(double x, double y, double yaw);

builtin_interfaces::msg::Time to_message_time(const rclcpp::Time & time);

std::chrono::milliseconds milliseconds_from_seconds(double seconds);

bool timestamp_is_current(
    const builtin_interfaces::msg::Time & stamp,
    const rclcpp::Time & now,
    double max_age_sec,
    double future_tolerance_sec);

}  // namespace auto_patrol::pose_utils
