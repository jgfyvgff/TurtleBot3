#include "auto_patrol/patrol_config.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace auto_patrol
{

void declare_patrol_parameters(rclcpp::Node & node)
{
    node.declare_parameter<std::string>("goal_frame", "map");
    node.declare_parameter<std::string>("amcl_pose_topic", "/amcl_pose");
    node.declare_parameter<std::string>("scan_topic", "/scan");
    node.declare_parameter<bool>("automatic_global_localization", true);
    node.declare_parameter<double>("localization_timeout_sec", 120.0);
    node.declare_parameter<double>(
        "localization_position_variance_threshold",
        0.25);
    node.declare_parameter<double>(
        "localization_yaw_variance_threshold",
        0.10);
    node.declare_parameter<bool>(
        "localization_exploration_enabled",
        true);
    node.declare_parameter<std::string>(
        "localization_cmd_vel_topic",
        "/cmd_vel_nav");
    node.declare_parameter<double>(
        "localization_exploration_angular_speed",
        0.20);
    node.declare_parameter<double>(
        "localization_exploration_max_duration_sec",
        40.0);
    node.declare_parameter<double>(
        "localization_settle_duration_sec",
        1.0);
    node.declare_parameter<bool>(
        "use_manual_initial_pose_fallback",
        false);
    node.declare_parameter<int>("max_retries", 1);
    node.declare_parameter<int>("stable_amcl_samples", 5);
    node.declare_parameter<double>("stable_position_tolerance", 0.05);
    node.declare_parameter<double>("stable_yaw_tolerance", 0.10);
    node.declare_parameter<double>("max_message_age_sec", 2.0);
    node.declare_parameter<double>("future_message_tolerance_sec", 0.5);
    node.declare_parameter<double>("initial_pose_x", 0.0);
    node.declare_parameter<double>("initial_pose_y", 0.0);
    node.declare_parameter<double>("initial_pose_yaw", 0.0);
    node.declare_parameter<int>("initial_pose_publish_count", 5);
    node.declare_parameter<double>("initial_pose_publish_period_sec", 1.0);
    node.declare_parameter<double>("initial_pose_wait_sec", 3.0);

    for (std::size_t index = 1; index <= 3; ++index) {
        node.declare_parameter<std::vector<double>>(
            "waypoint_" + std::to_string(index),
            std::vector<double>{0.0, 0.0, 0.0});
    }
}

PatrolConfig load_patrol_config(const rclcpp::Node & node)
{
    PatrolConfig config;
    config.goal_frame = node.get_parameter("goal_frame").as_string();
    config.amcl_pose_topic = node.get_parameter("amcl_pose_topic").as_string();
    config.scan_topic = node.get_parameter("scan_topic").as_string();
    config.automatic_global_localization =
        node.get_parameter("automatic_global_localization").as_bool();
    config.localization_timeout_sec =
        node.get_parameter("localization_timeout_sec").as_double();
    config.localization_position_variance_threshold =
        node.get_parameter("localization_position_variance_threshold").as_double();
    config.localization_yaw_variance_threshold =
        node.get_parameter("localization_yaw_variance_threshold").as_double();
    config.localization_exploration_enabled =
        node.get_parameter("localization_exploration_enabled").as_bool();
    config.localization_cmd_vel_topic =
        node.get_parameter("localization_cmd_vel_topic").as_string();
    config.localization_exploration_angular_speed =
        node.get_parameter("localization_exploration_angular_speed").as_double();
    config.localization_exploration_max_duration_sec =
        node.get_parameter(
        "localization_exploration_max_duration_sec").as_double();
    config.localization_settle_duration_sec =
        node.get_parameter("localization_settle_duration_sec").as_double();
    config.use_manual_initial_pose_fallback =
        node.get_parameter("use_manual_initial_pose_fallback").as_bool();
    config.max_retries = static_cast<int>(node.get_parameter("max_retries").as_int());
    config.stable_amcl_samples = static_cast<int>(
        node.get_parameter("stable_amcl_samples").as_int());
    config.stable_position_tolerance =
        node.get_parameter("stable_position_tolerance").as_double();
    config.stable_yaw_tolerance =
        node.get_parameter("stable_yaw_tolerance").as_double();
    config.max_message_age_sec = node.get_parameter("max_message_age_sec").as_double();
    config.future_message_tolerance_sec =
        node.get_parameter("future_message_tolerance_sec").as_double();
    config.initial_pose_x = node.get_parameter("initial_pose_x").as_double();
    config.initial_pose_y = node.get_parameter("initial_pose_y").as_double();
    config.initial_pose_yaw = node.get_parameter("initial_pose_yaw").as_double();
    config.initial_pose_publish_count = static_cast<int>(
        node.get_parameter("initial_pose_publish_count").as_int());
    config.initial_pose_publish_period_sec =
        node.get_parameter("initial_pose_publish_period_sec").as_double();
    config.initial_pose_wait_sec =
        node.get_parameter("initial_pose_wait_sec").as_double();

    config.waypoints.reserve(3);
    for (std::size_t index = 1; index <= 3; ++index) {
        const std::string parameter_name = "waypoint_" + std::to_string(index);
        const std::vector<double> values =
            node.get_parameter(parameter_name).as_double_array();

        if (values.size() != 3) {
            throw std::runtime_error(
                parameter_name + " must contain exactly [x, y, yaw].");
        }

        config.waypoints.push_back(Waypoint{values[0], values[1], values[2]});
    }

    return config;
}

void validate_patrol_config(const PatrolConfig & config)
{
    if (config.goal_frame.empty()) {
        throw std::runtime_error("goal_frame must not be empty.");
    }
    if (config.amcl_pose_topic.empty()) {
        throw std::runtime_error("amcl_pose_topic must not be empty.");
    }
    if (config.scan_topic.empty()) {
        throw std::runtime_error("scan_topic must not be empty.");
    }
    if (config.localization_timeout_sec <= 0.0) {
        throw std::runtime_error("localization_timeout_sec must be positive.");
    }
    if (config.localization_position_variance_threshold <= 0.0) {
        throw std::runtime_error(
            "localization_position_variance_threshold must be positive.");
    }
    if (config.localization_yaw_variance_threshold <= 0.0) {
        throw std::runtime_error(
            "localization_yaw_variance_threshold must be positive.");
    }
    if (config.localization_exploration_enabled &&
        config.localization_cmd_vel_topic.empty())
    {
        throw std::runtime_error(
            "localization_cmd_vel_topic must not be empty when exploration is enabled.");
    }
    if (!std::isfinite(config.localization_exploration_angular_speed) ||
        config.localization_exploration_angular_speed == 0.0)
    {
        throw std::runtime_error(
            "localization_exploration_angular_speed must be finite and non-zero.");
    }
    if (!std::isfinite(config.localization_exploration_max_duration_sec) ||
        config.localization_exploration_max_duration_sec <= 0.0)
    {
        throw std::runtime_error(
            "localization_exploration_max_duration_sec must be finite and positive.");
    }
    if (!std::isfinite(config.localization_settle_duration_sec) ||
        config.localization_settle_duration_sec <= 0.0)
    {
        throw std::runtime_error(
            "localization_settle_duration_sec must be finite and positive.");
    }
    if (config.automatic_global_localization ==
        config.use_manual_initial_pose_fallback)
    {
        throw std::runtime_error(
            "select exactly one localization bootstrap mode.");
    }
    if (config.max_retries < 0) {
        throw std::runtime_error("max_retries must be non-negative.");
    }
    if (config.stable_amcl_samples < 1) {
        throw std::runtime_error("stable_amcl_samples must be at least 1.");
    }
    if (config.stable_position_tolerance <= 0.0) {
        throw std::runtime_error("stable_position_tolerance must be positive.");
    }
    if (config.stable_yaw_tolerance <= 0.0) {
        throw std::runtime_error("stable_yaw_tolerance must be positive.");
    }
    if (config.max_message_age_sec <= 0.0) {
        throw std::runtime_error("max_message_age_sec must be positive.");
    }
    if (config.future_message_tolerance_sec < 0.0) {
        throw std::runtime_error(
            "future_message_tolerance_sec must not be negative.");
    }
    if (config.initial_pose_publish_count < 1) {
        throw std::runtime_error(
            "initial_pose_publish_count must be at least 1.");
    }
    if (config.initial_pose_publish_period_sec <= 0.0) {
        throw std::runtime_error(
            "initial_pose_publish_period_sec must be positive.");
    }
    if (config.initial_pose_wait_sec < 0.0) {
        throw std::runtime_error("initial_pose_wait_sec must not be negative.");
    }
    if (config.waypoints.size() != 3) {
        throw std::runtime_error("exactly three waypoints are required.");
    }

    const auto check_finite = [](double value, const std::string & name) {
        if (!std::isfinite(value)) {
            throw std::runtime_error(name + " must be finite.");
        }
    };

    check_finite(config.initial_pose_x, "initial_pose_x");
    check_finite(config.initial_pose_y, "initial_pose_y");
    check_finite(config.initial_pose_yaw, "initial_pose_yaw");

    for (std::size_t index = 0; index < config.waypoints.size(); ++index) {
        const std::string prefix = "waypoint_" + std::to_string(index + 1);
        check_finite(config.waypoints[index].x, prefix + "[0]");
        check_finite(config.waypoints[index].y, prefix + "[1]");
        check_finite(config.waypoints[index].yaw, prefix + "[2]");
    }
}

}  // namespace auto_patrol
