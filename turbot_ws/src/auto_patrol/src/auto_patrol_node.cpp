#include "auto_patrol/auto_patrol_node.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

namespace auto_patrol
{

AutoPatrolNode::AutoPatrolNode()
    : Node("auto_patrol")
{
    declare_patrol_parameters(*this);
    config_ = load_patrol_config(*this);
    validate_patrol_config(config_);

    localization_monitor_ = std::make_unique<LocalizationMonitor>(config_);
    if (!config_.automatic_global_localization &&
        config_.use_manual_initial_pose_fallback)
    {
        initial_pose_publisher_ = std::make_unique<InitialPosePublisher>(
            *this,
            config_);
    }
    if (config_.automatic_global_localization) {
        localization_bootstrapper_ =
            std::make_unique<LocalizationBootstrapper>(
            *this,
            config_,
            *localization_monitor_,
            [this](bool success) {
                handle_localization_finished(success);
            });
    }
    patrol_controller_ = std::make_unique<PatrolController>(
        *this,
        config_,
        [this](bool success) {
            handle_patrol_finished(success);
        });

    amcl_pose_subscription_ = create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        config_.amcl_pose_topic,
        rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
            handle_amcl_pose(message);
        });

    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        config_.scan_topic,
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
            handle_scan(message);
        });

    startup_timer_ = create_wall_timer(500ms, [this]() { startup_tick(); });

    RCLCPP_INFO(
        get_logger(),
        "auto_patrol started with %zu waypoints.",
        config_.waypoints.size());
}

AutoPatrolNode::~AutoPatrolNode()
{
    stop_components();
    stop_timers();
}

int AutoPatrolNode::exit_code() const noexcept
{
    return exit_code_;
}

void AutoPatrolNode::handle_scan(
    const sensor_msgs::msg::LaserScan::SharedPtr message)
{
    if (!message) {
        return;
    }
    const bool current = localization_monitor_->update_scan(*message, now());
    if (!current) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "LaserScan timestamp is stale or inconsistent.");
    }
}

void AutoPatrolNode::handle_amcl_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
{
    if (!message) {
        return;
    }
    const bool current = localization_monitor_->update_amcl_pose(*message, now());
    if (!current) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "AMCL pose timestamp is stale or inconsistent.");
    }
}

void AutoPatrolNode::startup_tick()
{
    if (startup_finished_ || finished_) {
        return;
    }

    if (!patrol_controller_->action_server_is_ready()) {
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Waiting for /navigate_to_pose Action Server...");
        return;
    }

    startup_finished_ = true;
    if (startup_timer_) {
        startup_timer_->cancel();
        startup_timer_.reset();
    }

    if (!config_.automatic_global_localization &&
        config_.use_manual_initial_pose_fallback)
    {
        initial_pose_publisher_->start([this]() { start_localization_wait(); });
    } else if (localization_bootstrapper_) {
        localization_bootstrapper_->start();
    } else {
        RCLCPP_ERROR(
            get_logger(),
            "No localization bootstrap mode is available.");
        handle_localization_finished(false);
    }
}

void AutoPatrolNode::handle_localization_finished(bool success)
{
    if (finished_) {
        return;
    }

    if (!success) {
        finished_ = true;
        exit_code_ = 1;
        stop_timers();
        if (patrol_controller_) {
            patrol_controller_->stop();
        }
        if (initial_pose_publisher_) {
            initial_pose_publisher_->stop();
        }
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(
        get_logger(),
        "Localization bootstrap completed; starting patrol.");
    patrol_controller_->start();
}

void AutoPatrolNode::start_localization_wait()
{
    if (finished_ || localization_timer_) {
        return;
    }

    localization_timer_ = create_wall_timer(500ms, [this]() {
        if (finished_) {
            return;
        }

        if (!localization_monitor_->is_ready(now())) {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Waiting for stable AMCL and current LaserScan...");
            return;
        }

        localization_timer_->cancel();
        localization_timer_.reset();
        RCLCPP_INFO(
            get_logger(),
            "Manual initial pose is localized with sufficient confidence.");
        patrol_controller_->start();
    });
}

void AutoPatrolNode::handle_patrol_finished(bool success)
{
    if (finished_) {
        return;
    }
    finished_ = true;
    exit_code_ = success ? 0 : 1;
    stop_timers();
    if (localization_bootstrapper_) {
        localization_bootstrapper_->stop();
    }
    if (initial_pose_publisher_) {
        initial_pose_publisher_->stop();
    }
    rclcpp::shutdown();
}

void AutoPatrolNode::stop_timers()
{
    if (startup_timer_) {
        startup_timer_->cancel();
        startup_timer_.reset();
    }
    if (localization_timer_) {
        localization_timer_->cancel();
        localization_timer_.reset();
    }
}

void AutoPatrolNode::stop_components()
{
    if (localization_bootstrapper_) {
        localization_bootstrapper_->stop();
    }
    if (patrol_controller_) {
        patrol_controller_->stop();
    }
    if (initial_pose_publisher_) {
        initial_pose_publisher_->stop();
    }
}

}  // namespace auto_patrol
