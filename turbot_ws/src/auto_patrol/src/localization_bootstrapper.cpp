#include "auto_patrol/localization_bootstrapper.hpp"

#include <chrono>
#include <exception>
#include <utility>

using namespace std::chrono_literals;

namespace auto_patrol
{

LocalizationBootstrapper::LocalizationBootstrapper(
    rclcpp::Node & node,
    PatrolConfig config,
    LocalizationMonitor & monitor,
    CompletionCallback on_complete)
    : node_(&node),
      config_(std::move(config)),
      monitor_(&monitor),
      on_complete_(std::move(on_complete))
{
    global_localization_client_ = node_->create_client<std_srvs::srv::Empty>(
        "/reinitialize_global_localization");
    if (config_.localization_exploration_enabled) {
        localization_cmd_vel_publisher_ =
            node_->create_publisher<geometry_msgs::msg::Twist>(
            config_.localization_cmd_vel_topic,
            rclcpp::QoS(10));
    }
}

LocalizationBootstrapper::~LocalizationBootstrapper()
{
    stop();
}

void LocalizationBootstrapper::start()
{
    if (state_ != State::IDLE) {
        return;
    }

    state_ = State::WAITING_FOR_GLOBAL_LOCALIZATION_SERVICE;
    started_at_ = std::chrono::steady_clock::now();
    service_timer_ = node_->create_wall_timer(500ms, [this]() {
        wait_for_service();
    });
}

void LocalizationBootstrapper::stop()
{
    if (state_ == State::STOPPED) {
        return;
    }

    state_ = State::STOPPED;
    cancel_timer(service_timer_);
    cancel_timer(convergence_timer_);
    cancel_timer(exploration_timer_);
    cancel_timer(settle_timer_);
    publish_zero_velocity();
    on_complete_ = {};
}

void LocalizationBootstrapper::wait_for_service()
{
    if (state_ != State::WAITING_FOR_GLOBAL_LOCALIZATION_SERVICE) {
        return;
    }

    const auto elapsed = std::chrono::steady_clock::now() - started_at_;
    if (elapsed > std::chrono::duration<double>(
            config_.localization_timeout_sec))
    {
        finish_failure(
            "Timed out waiting for /reinitialize_global_localization.");
        return;
    }

    if (!global_localization_client_->service_is_ready()) {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "Waiting for AMCL global localization service...");
        return;
    }

    cancel_timer(service_timer_);
    request_global_localization();
}

void LocalizationBootstrapper::request_global_localization()
{
    if (state_ != State::WAITING_FOR_GLOBAL_LOCALIZATION_SERVICE) {
        return;
    }

    // 服务会重新分布粒子，服务调用前的稳定样本不能用于新的定位周期。
    monitor_->reset_for_relocalization();
    state_ = State::REQUESTING_GLOBAL_LOCALIZATION;
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    global_localization_client_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
            if (state_ != State::REQUESTING_GLOBAL_LOCALIZATION) {
                return;
            }

            try {
                (void)future.get();
            } catch (const std::exception & exception) {
                finish_failure(
                    std::string("AMCL global localization failed: ") +
                    exception.what());
                return;
            }

            if (config_.localization_exploration_enabled) {
                state_ = State::EXPLORING;
                exploration_started_at_ = std::chrono::steady_clock::now();
                exploration_timer_ = node_->create_wall_timer(500ms, [this]() {
                    exploration_tick();
                });
                RCLCPP_INFO(
                    node_->get_logger(),
                    "AMCL global localization requested; starting automatic exploration.");
                return;
            }

            state_ = State::WAITING_FOR_CONVERGENCE;
            convergence_timer_ = node_->create_wall_timer(500ms, [this]() {
                check_convergence();
            });

            RCLCPP_INFO(
                node_->get_logger(),
                "AMCL global localization requested; waiting for convergence.");
        });
}

void LocalizationBootstrapper::check_convergence()
{
    if (state_ != State::WAITING_FOR_CONVERGENCE) {
        return;
    }

    const auto elapsed = std::chrono::steady_clock::now() - started_at_;
    if (elapsed > std::chrono::duration<double>(
            config_.localization_timeout_sec))
    {
        finish_failure("AMCL localization did not converge before timeout.");
        return;
    }

    if (!monitor_->is_ready(node_->now())) {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "Waiting for AMCL pose stability and covariance confidence...");
        return;
    }

    finish_success();
}

void LocalizationBootstrapper::finish_success()
{
    if (state_ != State::WAITING_FOR_CONVERGENCE &&
        state_ != State::WAITING_FOR_SETTLE)
    {
        return;
    }

    state_ = State::SUCCEEDED;
    cancel_timer(convergence_timer_);
    cancel_timer(exploration_timer_);
    cancel_timer(settle_timer_);
    publish_zero_velocity();
    RCLCPP_INFO(
        node_->get_logger(),
        "AMCL automatically localized the robot with sufficient confidence.");
    auto callback = std::move(on_complete_);
    if (callback) {
        callback(true);
    }
}

void LocalizationBootstrapper::finish_failure(const std::string & reason)
{
    if (state_ == State::FAILED || state_ == State::STOPPED ||
        state_ == State::SUCCEEDED)
    {
        return;
    }

    state_ = State::FAILED;
    cancel_timer(service_timer_);
    cancel_timer(convergence_timer_);
    cancel_timer(exploration_timer_);
    cancel_timer(settle_timer_);
    publish_zero_velocity();
    RCLCPP_ERROR(node_->get_logger(), "%s", reason.c_str());
    auto callback = std::move(on_complete_);
    if (callback) {
        callback(false);
    }
}

void LocalizationBootstrapper::exploration_tick()
{
    if (state_ != State::EXPLORING) {
        return;
    }

    const auto total_elapsed = std::chrono::steady_clock::now() - started_at_;
    if (total_elapsed > std::chrono::duration<double>(
            config_.localization_timeout_sec))
    {
        finish_failure("AMCL localization did not converge before timeout.");
        return;
    }

    if (monitor_->relocalization_is_ready(node_->now())) {
        RCLCPP_INFO(
            node_->get_logger(),
            "AMCL confidence reached; stopping exploration for final confirmation.");
        begin_settling();
        return;
    }

    const auto elapsed = std::chrono::steady_clock::now() -
        exploration_started_at_;
    if (elapsed > std::chrono::duration<double>(
            config_.localization_exploration_max_duration_sec))
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "AMCL exploration duration reached; stopping for final confirmation.");
        begin_settling();
        return;
    }

    geometry_msgs::msg::Twist command;
    // 低速原地旋转优先获取不同方向的激光观测，避免定位阶段盲目前进。
    command.angular.z = config_.localization_exploration_angular_speed;
    localization_cmd_vel_publisher_->publish(command);
}

void LocalizationBootstrapper::begin_settling()
{
    if (state_ != State::EXPLORING) {
        return;
    }

    // 必须先取消旋转 Timer 并发送零速度，之后的协方差才对应静止后的最终定位状态。
    cancel_timer(exploration_timer_);
    publish_zero_velocity();
    state_ = State::WAITING_FOR_SETTLE;
    settle_started_at_ = std::chrono::steady_clock::now();
    settle_timer_ = node_->create_wall_timer(200ms, [this]() {
        check_settled_localization();
    });
}

void LocalizationBootstrapper::check_settled_localization()
{
    if (state_ != State::WAITING_FOR_SETTLE) {
        return;
    }

    const auto total_elapsed = std::chrono::steady_clock::now() - started_at_;
    if (total_elapsed > std::chrono::duration<double>(
            config_.localization_timeout_sec))
    {
        finish_failure("AMCL localization did not converge before timeout.");
        return;
    }

    const auto settle_elapsed = std::chrono::steady_clock::now() -
        settle_started_at_;
    if (settle_elapsed < std::chrono::duration<double>(
            config_.localization_settle_duration_sec))
    {
        return;
    }

    if (monitor_->confidence_is_sufficient(node_->now())) {
        finish_success();
        return;
    }

    finish_failure("AMCL localization did not converge after exploration.");
}

void LocalizationBootstrapper::publish_zero_velocity()
{
    if (localization_cmd_vel_publisher_) {
        localization_cmd_vel_publisher_->publish(
            geometry_msgs::msg::Twist{});
    }
}

void LocalizationBootstrapper::cancel_timer(
    rclcpp::TimerBase::SharedPtr & timer)
{
    if (timer) {
        timer->cancel();
        timer.reset();
    }
}

}  // namespace auto_patrol
