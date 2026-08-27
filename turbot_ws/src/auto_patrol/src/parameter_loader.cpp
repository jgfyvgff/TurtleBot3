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
    node.declare_parameter<std::string>("localization_map_topic", "/map");
    node.declare_parameter<std::string>(
        "localization_base_frame", "base_footprint");
    node.declare_parameter<bool>("automatic_global_localization", true);
    node.declare_parameter<double>("localization_timeout_sec", 120.0);
    node.declare_parameter<double>(
        "localization_position_variance_threshold",
        0.25);
    node.declare_parameter<double>(
        "localization_yaw_variance_threshold",
        0.10);
    node.declare_parameter<std::string>(
        "localization_global_localization_service",
        "/reinitialize_global_localization");
    node.declare_parameter<std::string>(
        "localization_nomotion_update_service",
        "/request_nomotion_update");
    node.declare_parameter<double>(
        "localization_nomotion_update_period_sec",
        0.5);
    node.declare_parameter<std::string>("localization_odom_frame", "odom");
    node.declare_parameter<double>(
        "localization_settle_duration_sec",
        1.0);
    node.declare_parameter<bool>("localization_scan_match_enabled", true);
    node.declare_parameter<double>(
        "localization_scan_match_coarse_step_m", 0.20);
    node.declare_parameter<double>(
        "localization_scan_match_coarse_yaw_step_rad", 0.261799);
    node.declare_parameter<double>(
        "localization_scan_match_refine_step_m", 0.05);
    node.declare_parameter<double>(
        "localization_scan_match_refine_yaw_step_rad", 0.034907);
    node.declare_parameter<int>("localization_scan_match_max_beams", 90);
    node.declare_parameter<double>(
        "localization_scan_match_max_range_m", 3.0);
    node.declare_parameter<int>(
        "localization_scan_match_free_space_max_beams", 12);
    node.declare_parameter<double>(
        "localization_scan_match_free_space_penalty_m", 0.25);
    node.declare_parameter<double>(
        "localization_scan_match_max_mean_error_m", 0.10);
    node.declare_parameter<double>(
        "localization_scan_match_min_margin_m", 0.05);
    node.declare_parameter<double>(
        "localization_scan_match_min_separation_m", 0.50);
    node.declare_parameter<double>(
        "localization_scan_match_min_yaw_separation_rad", 0.523599);
    node.declare_parameter<double>(
        "localization_scan_match_pose_tolerance_m", 0.20);
    node.declare_parameter<double>(
        "localization_scan_match_yaw_tolerance_rad", 0.20);
    node.declare_parameter<int>(
        "localization_scan_match_max_initial_pose_retries", 1);
    node.declare_parameter<double>("costmap_clear_timeout_sec", 5.0);
    node.declare_parameter<double>("costmap_clear_settle_duration_sec", 2.0);
    node.declare_parameter<std::string>(
        "global_costmap_clear_service",
        "/global_costmap/clear_entirely_global_costmap");
    node.declare_parameter<std::string>(
        "local_costmap_clear_service",
        "/local_costmap/clear_entirely_local_costmap");
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
    config.localization_map_topic =
        node.get_parameter("localization_map_topic").as_string();
    config.localization_base_frame =
        node.get_parameter("localization_base_frame").as_string();
    config.automatic_global_localization =
        node.get_parameter("automatic_global_localization").as_bool();
    config.localization_timeout_sec =
        node.get_parameter("localization_timeout_sec").as_double();
    config.localization_position_variance_threshold =
        node.get_parameter("localization_position_variance_threshold").as_double();
    config.localization_yaw_variance_threshold =
        node.get_parameter("localization_yaw_variance_threshold").as_double();
    config.localization_global_localization_service =
        node.get_parameter("localization_global_localization_service").as_string();
    config.localization_nomotion_update_service =
        node.get_parameter("localization_nomotion_update_service").as_string();
    config.localization_nomotion_update_period_sec =
        node.get_parameter("localization_nomotion_update_period_sec").as_double();
    config.localization_odom_frame =
        node.get_parameter("localization_odom_frame").as_string();
    config.localization_settle_duration_sec =
        node.get_parameter("localization_settle_duration_sec").as_double();
    config.localization_scan_match_enabled =
        node.get_parameter("localization_scan_match_enabled").as_bool();
    config.localization_scan_match_coarse_step_m =
        node.get_parameter("localization_scan_match_coarse_step_m").as_double();
    config.localization_scan_match_coarse_yaw_step_rad =
        node.get_parameter("localization_scan_match_coarse_yaw_step_rad").as_double();
    config.localization_scan_match_refine_step_m =
        node.get_parameter("localization_scan_match_refine_step_m").as_double();
    config.localization_scan_match_refine_yaw_step_rad =
        node.get_parameter("localization_scan_match_refine_yaw_step_rad").as_double();
    config.localization_scan_match_max_beams = static_cast<int>(
        node.get_parameter("localization_scan_match_max_beams").as_int());
    config.localization_scan_match_max_range_m =
        node.get_parameter("localization_scan_match_max_range_m").as_double();
    config.localization_scan_match_free_space_max_beams = static_cast<int>(
        node.get_parameter("localization_scan_match_free_space_max_beams").as_int());
    config.localization_scan_match_free_space_penalty_m =
        node.get_parameter("localization_scan_match_free_space_penalty_m").as_double();
    config.localization_scan_match_max_mean_error_m =
        node.get_parameter("localization_scan_match_max_mean_error_m").as_double();
    config.localization_scan_match_min_margin_m =
        node.get_parameter("localization_scan_match_min_margin_m").as_double();
    config.localization_scan_match_min_separation_m =
        node.get_parameter("localization_scan_match_min_separation_m").as_double();
    config.localization_scan_match_min_yaw_separation_rad =
        node.get_parameter(
            "localization_scan_match_min_yaw_separation_rad").as_double();
    config.localization_scan_match_pose_tolerance_m =
        node.get_parameter("localization_scan_match_pose_tolerance_m").as_double();
    config.localization_scan_match_yaw_tolerance_rad =
        node.get_parameter("localization_scan_match_yaw_tolerance_rad").as_double();
    config.localization_scan_match_max_initial_pose_retries = static_cast<int>(
        node.get_parameter(
            "localization_scan_match_max_initial_pose_retries").as_int());
    config.costmap_clear_timeout_sec =
        node.get_parameter("costmap_clear_timeout_sec").as_double();
    config.costmap_clear_settle_duration_sec =
        node.get_parameter("costmap_clear_settle_duration_sec").as_double();
    config.global_costmap_clear_service =
        node.get_parameter("global_costmap_clear_service").as_string();
    config.local_costmap_clear_service =
        node.get_parameter("local_costmap_clear_service").as_string();
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
    if (config.localization_map_topic.empty()) {
        throw std::runtime_error("localization_map_topic must not be empty.");
    }
    if (config.localization_base_frame.empty()) {
        throw std::runtime_error("localization_base_frame must not be empty.");
    }
    if (!std::isfinite(config.localization_timeout_sec) ||
        config.localization_timeout_sec <= 0.0)
    {
        throw std::runtime_error("localization_timeout_sec must be positive.");
    }
    if (!std::isfinite(config.localization_position_variance_threshold) ||
        config.localization_position_variance_threshold <= 0.0)
    {
        throw std::runtime_error(
            "localization_position_variance_threshold must be positive.");
    }
    if (!std::isfinite(config.localization_yaw_variance_threshold) ||
        config.localization_yaw_variance_threshold <= 0.0)
    {
        throw std::runtime_error(
            "localization_yaw_variance_threshold must be positive.");
    }
    if (config.localization_global_localization_service.empty() ||
        config.localization_nomotion_update_service.empty())
    {
        throw std::runtime_error(
            "AMCL localization service names must not be empty.");
    }
    if (!std::isfinite(config.localization_nomotion_update_period_sec) ||
        config.localization_nomotion_update_period_sec < 0.05)
    {
        throw std::runtime_error(
            "localization_nomotion_update_period_sec must be finite and at least 0.05.");
    }
    if (config.localization_odom_frame.empty()) {
        throw std::runtime_error("localization_odom_frame must not be empty.");
    }
    if (!std::isfinite(config.localization_settle_duration_sec) ||
        config.localization_settle_duration_sec <= 0.0)
    {
        throw std::runtime_error(
            "localization_settle_duration_sec must be finite and positive.");
    }
    const auto require_positive_finite = [](double value, const std::string & name) {
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::runtime_error(name + " must be finite and positive.");
        }
    };
    require_positive_finite(
        config.localization_scan_match_coarse_step_m,
        "localization_scan_match_coarse_step_m");
    require_positive_finite(
        config.localization_scan_match_coarse_yaw_step_rad,
        "localization_scan_match_coarse_yaw_step_rad");
    require_positive_finite(
        config.localization_scan_match_refine_step_m,
        "localization_scan_match_refine_step_m");
    require_positive_finite(
        config.localization_scan_match_refine_yaw_step_rad,
        "localization_scan_match_refine_yaw_step_rad");
    require_positive_finite(
        config.localization_scan_match_max_range_m,
        "localization_scan_match_max_range_m");
    require_positive_finite(
        config.localization_scan_match_free_space_penalty_m,
        "localization_scan_match_free_space_penalty_m");
    require_positive_finite(
        config.localization_scan_match_max_mean_error_m,
        "localization_scan_match_max_mean_error_m");
    require_positive_finite(
        config.localization_scan_match_min_margin_m,
        "localization_scan_match_min_margin_m");
    require_positive_finite(
        config.localization_scan_match_min_separation_m,
        "localization_scan_match_min_separation_m");
    require_positive_finite(
        config.localization_scan_match_min_yaw_separation_rad,
        "localization_scan_match_min_yaw_separation_rad");
    require_positive_finite(
        config.localization_scan_match_pose_tolerance_m,
        "localization_scan_match_pose_tolerance_m");
    require_positive_finite(
        config.localization_scan_match_yaw_tolerance_rad,
        "localization_scan_match_yaw_tolerance_rad");
    if (config.localization_scan_match_max_beams < 3) {
        throw std::runtime_error(
            "localization_scan_match_max_beams must be at least 3.");
    }
    if (config.localization_scan_match_free_space_max_beams < 3) {
        throw std::runtime_error(
            "localization_scan_match_free_space_max_beams must be at least 3.");
    }
    if (config.localization_scan_match_max_initial_pose_retries < 0) {
        throw std::runtime_error(
            "localization_scan_match_max_initial_pose_retries must be non-negative.");
    }
    if (!std::isfinite(config.costmap_clear_timeout_sec) ||
        config.costmap_clear_timeout_sec <= 0.0)
    {
        throw std::runtime_error(
            "costmap_clear_timeout_sec must be finite and positive.");
    }
    if (!std::isfinite(config.costmap_clear_settle_duration_sec) ||
        config.costmap_clear_settle_duration_sec < 0.0)
    {
        throw std::runtime_error(
            "costmap_clear_settle_duration_sec must be finite and non-negative.");
    }
    if (config.global_costmap_clear_service.empty() ||
        config.local_costmap_clear_service.empty())
    {
        throw std::runtime_error("costmap clear service names must not be empty.");
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
