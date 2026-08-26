#include "auto_patrol/localization_bootstrapper.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <utility>

#include "auto_patrol/localization_safety.hpp"
#include "auto_patrol/pose_utils.hpp"

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
    cancel_timer(translation_candidate_timer_);
    cancel_timer(translation_timer_);
    cancel_timer(second_localization_timer_);
    cancel_timer(settle_timer_);
    publish_zero_velocity();
    on_complete_ = {};
}

void LocalizationBootstrapper::update_odometry(
    const nav_msgs::msg::Odometry & message,
    const rclcpp::Time & now)
{
    const auto & position = message.pose.pose.position;
    if (!pose_utils::timestamp_is_current(
            message.header.stamp,
            now,
            config_.max_message_age_sec,
            config_.future_message_tolerance_sec) ||
        !std::isfinite(position.x) || !std::isfinite(position.y))
    {
        latest_odometry_.reset();
        return;
    }

    latest_odometry_ = OdometrySample{
        position.x,
        position.y,
        message.header.stamp};
}

void LocalizationBootstrapper::wait_for_service()
{
    if (state_ != State::WAITING_FOR_GLOBAL_LOCALIZATION_SERVICE) {
        return;
    }

    if (localization_timeout_expired()) {
        finish_failure("Timed out waiting for /reinitialize_global_localization.");
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
    latest_odometry_.reset();
    translation_start_.reset();
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

    if (localization_timeout_expired()) {
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

void LocalizationBootstrapper::exploration_tick()
{
    if (state_ != State::EXPLORING) {
        return;
    }

    if (localization_timeout_expired()) {
        finish_failure("AMCL localization did not converge before timeout.");
        return;
    }

    if (monitor_->relocalization_is_ready(node_->now())) {
        begin_translation_candidate();
        return;
    }

    const auto elapsed = std::chrono::steady_clock::now() -
        exploration_started_at_;
    if (elapsed > std::chrono::duration<double>(
            config_.localization_exploration_max_duration_sec))
    {
        finish_failure("AMCL localization exploration did not reach confidence.");
        return;
    }

    publish_angular_velocity();
}

void LocalizationBootstrapper::begin_translation_candidate()
{
    if (state_ != State::EXPLORING) {
        return;
    }

    // 第一次收敛只作为候选。先停下旋转，再寻找有前方净空的验证方向。
    cancel_timer(exploration_timer_);
    publish_zero_velocity();
    state_ = State::WAITING_FOR_TRANSLATION_CANDIDATE;
    translation_candidate_timer_ = node_->create_wall_timer(200ms, [this]() {
        translation_candidate_tick();
    });
    RCLCPP_INFO(
        node_->get_logger(),
        "AMCL confidence reached; checking front clearance for translation validation.");
}

void LocalizationBootstrapper::translation_candidate_tick()
{
    if (state_ != State::WAITING_FOR_TRANSLATION_CANDIDATE) {
        return;
    }

    if (localization_timeout_expired()) {
        finish_failure("AMCL localization validation did not finish before timeout.");
        return;
    }

    if (!monitor_->relocalization_is_ready(node_->now())) {
        // 置信度回落时继续旋转获取新观测，但绝不在低置信度状态下前进。
        publish_angular_velocity();
        return;
    }

    // 起步时必须保证完整验证路径后仍保留最小安全净空，不能只检查当前位置。
    const double required_start_clearance =
        config_.localization_exploration_min_front_clearance +
        config_.localization_exploration_translation_distance;
    if (!monitor_->front_clearance_is_safe(
            node_->now(),
            required_start_clearance,
            config_.localization_exploration_front_sector_half_angle))
    {
        // 找不到安全正前方时只允许旋转，避免把定位验证变成碰撞风险。
        publish_angular_velocity();
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "Waiting for a front LaserScan sector with safe translation clearance...");
        return;
    }

    if (!odometry_is_current(node_->now())) {
        publish_zero_velocity();
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "Waiting for current odometry before localization validation.");
        return;
    }

    start_translation();
}

void LocalizationBootstrapper::start_translation()
{
    if (state_ != State::WAITING_FOR_TRANSLATION_CANDIDATE ||
        !latest_odometry_.has_value())
    {
        return;
    }

    // 从此刻起的 AMCL 样本才属于主动验证证据；LaserScan 仍保留以持续检查净空。
    monitor_->begin_translation_validation();
    cancel_timer(translation_candidate_timer_);
    translation_start_ = *latest_odometry_;
    translation_started_at_ = std::chrono::steady_clock::now();
    state_ = State::TRANSLATING_FOR_VALIDATION;
    translation_timer_ = node_->create_wall_timer(100ms, [this]() {
        translation_tick();
    });
    RCLCPP_INFO(
        node_->get_logger(),
        "Starting %.3f m localization validation translation at %.3f m/s.",
        config_.localization_exploration_translation_distance,
        config_.localization_exploration_linear_speed);
    publish_linear_velocity();
}

void LocalizationBootstrapper::translation_tick()
{
    if (state_ != State::TRANSLATING_FOR_VALIDATION) {
        return;
    }

    if (localization_timeout_expired()) {
        finish_failure("AMCL localization validation did not finish before timeout.");
        return;
    }

    if (!monitor_->front_clearance_is_safe(
            node_->now(),
            config_.localization_exploration_min_front_clearance,
            config_.localization_exploration_front_sector_half_angle))
    {
        // 动态障碍或定位误差可能使净空在平移中下降；停止后换方向，不带风险继续前进。
        resume_translation_candidate();
        return;
    }

    if (!odometry_is_current(node_->now()) || !latest_odometry_.has_value() ||
        !translation_start_.has_value())
    {
        finish_failure("Stopped localization validation because odometry is stale.");
        return;
    }

    const double distance = localization_safety::planar_distance(
        translation_start_->x,
        translation_start_->y,
        latest_odometry_->x,
        latest_odometry_->y);
    if (distance >= config_.localization_exploration_translation_distance) {
        cancel_timer(translation_timer_);
        publish_zero_velocity();
        begin_second_localization();
        return;
    }

    const auto elapsed = std::chrono::steady_clock::now() - translation_started_at_;
    if (elapsed > std::chrono::duration<double>(
            config_.localization_exploration_translation_timeout_sec))
    {
        finish_failure("Localization validation translation timed out before reaching its distance.");
        return;
    }

    publish_linear_velocity();
}

void LocalizationBootstrapper::resume_translation_candidate()
{
    if (state_ != State::TRANSLATING_FOR_VALIDATION) {
        return;
    }

    cancel_timer(translation_timer_);
    publish_zero_velocity();
    translation_start_.reset();
    state_ = State::WAITING_FOR_TRANSLATION_CANDIDATE;
    translation_candidate_timer_ = node_->create_wall_timer(200ms, [this]() {
        translation_candidate_tick();
    });
    RCLCPP_WARN(
        node_->get_logger(),
        "Front clearance became unsafe during validation; rotating to choose a new direction.");
}

void LocalizationBootstrapper::begin_second_localization()
{
    if (state_ != State::TRANSLATING_FOR_VALIDATION) {
        return;
    }

    // 不能在平移完成后清空样本：AMCL 常只在运动时更新，平移过程已经产生有效新证据。
    state_ = State::WAITING_FOR_SECOND_LOCALIZATION;
    second_localization_timer_ = node_->create_wall_timer(200ms, [this]() {
        check_second_localization();
    });
    RCLCPP_INFO(
        node_->get_logger(),
        "Translation validation completed; collecting fresh AMCL evidence.");
}

void LocalizationBootstrapper::check_second_localization()
{
    if (state_ != State::WAITING_FOR_SECOND_LOCALIZATION) {
        return;
    }

    if (localization_timeout_expired()) {
        finish_failure("AMCL did not reconfirm localization after translation.");
        return;
    }

    if (!monitor_->relocalization_is_ready(node_->now())) {
        // 仅原地旋转以触发 update_min_a 对应的 AMCL 更新，不再进行未知位置下的前进。
        publish_angular_velocity();
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "Rotating safely to collect fresh AMCL samples after translation validation...");
        return;
    }

    begin_settling();
}

void LocalizationBootstrapper::begin_settling()
{
    if (state_ != State::WAITING_FOR_SECOND_LOCALIZATION) {
        return;
    }

    // 前一状态已经收集到连续低协方差的新证据；停止后不应要求 AMCL 继续发布位姿。
    cancel_timer(second_localization_timer_);
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

    if (localization_timeout_expired()) {
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

    finish_failure("AMCL localization became stale or lost confidence during final confirmation.");
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
    cancel_timer(translation_candidate_timer_);
    cancel_timer(translation_timer_);
    cancel_timer(second_localization_timer_);
    cancel_timer(settle_timer_);
    publish_zero_velocity();
    RCLCPP_INFO(
        node_->get_logger(),
        "AMCL automatically localized the robot with translation validation.");
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
    cancel_timer(translation_candidate_timer_);
    cancel_timer(translation_timer_);
    cancel_timer(second_localization_timer_);
    cancel_timer(settle_timer_);
    publish_zero_velocity();
    RCLCPP_ERROR(node_->get_logger(), "%s", reason.c_str());
    auto callback = std::move(on_complete_);
    if (callback) {
        callback(false);
    }
}

void LocalizationBootstrapper::publish_angular_velocity()
{
    if (!localization_cmd_vel_publisher_) {
        return;
    }

    geometry_msgs::msg::Twist command;
    // 旋转优先获得不同朝向的扫描；线速度始终为零，避免未知位置下盲目前进。
    command.angular.z = config_.localization_exploration_angular_speed;
    localization_cmd_vel_publisher_->publish(command);
}

void LocalizationBootstrapper::publish_linear_velocity()
{
    if (!localization_cmd_vel_publisher_) {
        return;
    }

    geometry_msgs::msg::Twist command;
    command.linear.x = config_.localization_exploration_linear_speed;
    localization_cmd_vel_publisher_->publish(command);
}

void LocalizationBootstrapper::publish_zero_velocity()
{
    if (localization_cmd_vel_publisher_) {
        localization_cmd_vel_publisher_->publish(geometry_msgs::msg::Twist{});
    }
}

bool LocalizationBootstrapper::localization_timeout_expired() const
{
    const auto elapsed = std::chrono::steady_clock::now() - started_at_;
    return elapsed > std::chrono::duration<double>(config_.localization_timeout_sec);
}

bool LocalizationBootstrapper::odometry_is_current(const rclcpp::Time & now) const
{
    return latest_odometry_.has_value() && pose_utils::timestamp_is_current(
        latest_odometry_->stamp,
        now,
        config_.max_message_age_sec,
        config_.future_message_tolerance_sec);
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
