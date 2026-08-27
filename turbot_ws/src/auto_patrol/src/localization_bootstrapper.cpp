#include "auto_patrol/localization_bootstrapper.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <sstream>
#include <utility>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/time.h"

#include "auto_patrol/pose_utils.hpp"

using namespace std::chrono_literals;

namespace auto_patrol
{

LocalizationBootstrapper::LocalizationBootstrapper(
    rclcpp::Node & node,
    PatrolConfig config,
    LocalizationMonitor & monitor,
    ScanMapMatcher & scan_map_matcher,
    InitialPosePublisher & initial_pose_publisher,
    CompletionCallback on_complete)
    : node_(&node),
      config_(std::move(config)),
      monitor_(&monitor),
      scan_map_matcher_(&scan_map_matcher),
      initial_pose_publisher_(&initial_pose_publisher),
      on_complete_(std::move(on_complete))
{
    global_localization_client_ = node_->create_client<std_srvs::srv::Empty>(
        config_.localization_global_localization_service);
    nomotion_update_client_ = node_->create_client<std_srvs::srv::Empty>(
        config_.localization_nomotion_update_service);

    // 监听器不自行 spin，避免和 AutoPatrolNode 的执行器竞争；生命周期由本组件统一管理。
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
        *tf_buffer_,
        node_,
        false);
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

    nomotion_update_confirmed_ = false;
    matched_initial_pose_.reset();
    matched_initial_pose_retry_count_ = 0;
    started_at_ = std::chrono::steady_clock::now();

    // 扫描匹配本身就是全局搜索。先让 AMCL 随机散布粒子会制造一个短暂、错误的
    // map -> odom，且不会为匹配器带来额外信息，因此这里直接使用静态地图和 LaserScan。
    if (config_.localization_scan_match_enabled) {
        monitor_->reset_for_relocalization();
        state_ = State::WAITING_FOR_SCAN_MATCH;
        scan_match_timer_ = node_->create_wall_timer(1s, [this]() {
            wait_for_scan_match();
        });
        RCLCPP_INFO(
            node_->get_logger(),
            "Starting independent scan-to-map global localization without startup motion.");
        wait_for_scan_match();
        return;
    }

    // 关闭扫描匹配时保留原有 AMCL 全局定位路径，便于在不具备静态地图输入的部署中兼容。
    state_ = State::WAITING_FOR_GLOBAL_LOCALIZATION_SERVICE;
    global_service_timer_ = node_->create_wall_timer(500ms, [this]() {
        wait_for_global_localization_service();
    });
}

void LocalizationBootstrapper::stop()
{
    if (state_ == State::STOPPED) {
        return;
    }

    state_ = State::STOPPED;
    nomotion_request_in_flight_ = false;
    nomotion_update_confirmed_ = false;
    matched_initial_pose_.reset();
    cancel_timer(global_service_timer_);
    cancel_timer(scan_match_timer_);
    cancel_timer(nomotion_service_timer_);
    cancel_timer(nomotion_update_timer_);
    cancel_timer(transform_timer_);
    cancel_timer(settle_timer_);
    if (initial_pose_publisher_) {
        initial_pose_publisher_->stop();
    }
    on_complete_ = {};
}

void LocalizationBootstrapper::wait_for_global_localization_service()
{
    if (state_ != State::WAITING_FOR_GLOBAL_LOCALIZATION_SERVICE) {
        return;
    }

    if (localization_timeout_expired()) {
        finish_failure(
            "Timed out waiting for AMCL global localization service.");
        return;
    }

    if (!global_localization_client_->service_is_ready()) {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "Waiting for AMCL global localization service %s...",
            config_.localization_global_localization_service.c_str());
        return;
    }

    cancel_timer(global_service_timer_);
    request_global_localization();
}

void LocalizationBootstrapper::request_global_localization()
{
    if (state_ != State::WAITING_FOR_GLOBAL_LOCALIZATION_SERVICE) {
        return;
    }

    // 服务会重新分布粒子，因此服务调用前的 scan、AMCL 和稳定计数不能继续作为证据。
    monitor_->reset_for_relocalization();
    nomotion_update_confirmed_ = false;
    matched_initial_pose_.reset();
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

            begin_nomotion_updates();
        });
}

void LocalizationBootstrapper::wait_for_scan_match()
{
    if (state_ != State::WAITING_FOR_SCAN_MATCH) {
        return;
    }

    if (localization_timeout_expired()) {
        finish_failure("Timed out waiting for an unambiguous scan-to-map localization.");
        return;
    }

    if (!scan_map_matcher_->has_map()) {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "Waiting for static map on %s...",
            config_.localization_map_topic.c_str());
        return;
    }
    if (!monitor_->scan_is_current(node_->now())) {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "Waiting for a current LaserScan before scan-to-map localization...");
        return;
    }

    const auto base_from_scan = base_from_scan_transform();
    if (!base_from_scan.has_value()) {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "Waiting for %s -> %s TF before scan-to-map localization...",
            config_.localization_base_frame.c_str(),
            scan_map_matcher_->scan_frame_id().c_str());
        return;
    }

    const ScanMatchResult result = scan_map_matcher_->find_global_match(
        *base_from_scan);
    if (!result.map_and_scan_ready) {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "Waiting for usable obstacle endpoints in LaserScan...");
        return;
    }
    if (!result.accepted) {
        if (result.runner_up_pose.has_value()) {
            const double margin_m = result.runner_up_mean_error_m -
                result.best_mean_error_m;
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                5000,
                "Scan-to-map candidate rejected: best [x=%.3f, y=%.3f, yaw=%.3f rad, "
                "total=%.3f m, endpoint=%.3f m, free-space=%.3f m], "
                "runner-up [x=%.3f, y=%.3f, yaw=%.3f rad, total=%.3f m, "
                "endpoint=%.3f m, free-space=%.3f m], margin=%.3f m "
                "(required %.3f m), separation=%.3f m / %.3f rad.",
                result.best_pose.x,
                result.best_pose.y,
                result.best_pose.yaw,
                result.best_mean_error_m,
                result.best_endpoint_mean_error_m,
                result.best_free_space_mean_penalty_m,
                result.runner_up_pose->x,
                result.runner_up_pose->y,
                result.runner_up_pose->yaw,
                result.runner_up_mean_error_m,
                result.runner_up_endpoint_mean_error_m,
                result.runner_up_free_space_mean_penalty_m,
                margin_m,
                config_.localization_scan_match_min_margin_m,
                result.runner_up_position_distance_m,
                result.runner_up_yaw_difference_rad);
        } else {
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                5000,
                "Scan-to-map candidate rejected: best [x=%.3f, y=%.3f, yaw=%.3f rad, "
                "total=%.3f m, endpoint=%.3f m, free-space=%.3f m]; no spatially "
                "separated runner-up was found.",
                result.best_pose.x,
                result.best_pose.y,
                result.best_pose.yaw,
                result.best_mean_error_m,
                result.best_endpoint_mean_error_m,
                result.best_free_space_mean_penalty_m);
        }
        return;
    }

    matched_initial_pose_ = result.best_pose;
    matched_initial_pose_retry_count_ = 0;
    RCLCPP_INFO(
        node_->get_logger(),
        "Scan-to-map candidate accepted: x=%.3f, y=%.3f, yaw=%.3f rad, "
        "total=%.3f m, endpoint=%.3f m, free-space=%.3f m.",
        result.best_pose.x,
        result.best_pose.y,
        result.best_pose.yaw,
        result.best_mean_error_m,
        result.best_endpoint_mean_error_m,
        result.best_free_space_mean_penalty_m);
    publish_matched_initial_pose();
}

void LocalizationBootstrapper::publish_matched_initial_pose()
{
    if (!matched_initial_pose_.has_value() || !initial_pose_publisher_) {
        finish_failure("Scan-to-map localization did not provide an initial pose.");
        return;
    }

    cancel_timer(scan_match_timer_);
    cancel_timer(nomotion_service_timer_);
    cancel_timer(nomotion_update_timer_);
    cancel_timer(transform_timer_);
    cancel_timer(settle_timer_);
    nomotion_request_in_flight_ = false;
    nomotion_update_confirmed_ = false;
    // 每次重发 /initialpose 都重新建立 AMCL 证据窗口，防止复用纠正前的错误局部模式。
    monitor_->reset_for_relocalization();
    state_ = State::PUBLISHING_MATCHED_INITIAL_POSE;
    initial_pose_publisher_->start(
        matched_initial_pose_->x,
        matched_initial_pose_->y,
        matched_initial_pose_->yaw,
        [this]() {
            if (state_ != State::PUBLISHING_MATCHED_INITIAL_POSE) {
                return;
            }
            begin_nomotion_updates();
        });
}

void LocalizationBootstrapper::begin_nomotion_updates()
{
    if (state_ != State::PUBLISHING_MATCHED_INITIAL_POSE &&
        state_ != State::REQUESTING_GLOBAL_LOCALIZATION)
    {
        return;
    }

    state_ = State::WAITING_FOR_NOMOTION_UPDATE_SERVICE;
    nomotion_service_timer_ = node_->create_wall_timer(500ms, [this]() {
        wait_for_nomotion_update_service();
    });
    RCLCPP_INFO(
        node_->get_logger(),
        "Waiting for AMCL no-motion update service after localization bootstrap.");
}

void LocalizationBootstrapper::wait_for_nomotion_update_service()
{
    if (state_ != State::WAITING_FOR_NOMOTION_UPDATE_SERVICE) {
        return;
    }

    if (localization_timeout_expired()) {
        finish_failure("Timed out waiting for AMCL no-motion update service.");
        return;
    }

    if (!nomotion_update_client_->service_is_ready()) {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "Waiting for AMCL no-motion update service...");
        return;
    }

    cancel_timer(nomotion_service_timer_);
    state_ = State::WAITING_FOR_CONVERGENCE;
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(config_.localization_nomotion_update_period_sec));
    nomotion_update_timer_ = node_->create_wall_timer(period, [this]() {
        no_motion_update_tick();
    });
    RCLCPP_INFO(
        node_->get_logger(),
        "AMCL no-motion update service is ready; requesting fresh localization evidence.");
    no_motion_update_tick();
}

void LocalizationBootstrapper::no_motion_update_tick()
{
    if (state_ != State::WAITING_FOR_CONVERGENCE) {
        return;
    }

    if (localization_timeout_expired()) {
        finish_failure("AMCL no-motion localization did not converge before timeout.");
        return;
    }

    if (nomotion_update_confirmed_ && monitor_->is_ready(node_->now())) {
        std::string reason;
        if (!amcl_matches_scan_map(reason)) {
            retry_matched_initial_pose(reason);
            return;
        }

        cancel_timer(nomotion_update_timer_);
        state_ = State::WAITING_FOR_MAP_TO_ODOM_TRANSFORM;
        transform_timer_ = node_->create_wall_timer(500ms, [this]() {
            check_map_to_odom_transform();
        });
        RCLCPP_INFO(
            node_->get_logger(),
            "AMCL agrees with the independent scan-map pose; verifying %s -> %s.",
            config_.goal_frame.c_str(),
            config_.localization_odom_frame.c_str());
        check_map_to_odom_transform();
        return;
    }

    if (nomotion_request_in_flight_) {
        return;
    }

    nomotion_request_in_flight_ = true;
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    nomotion_update_client_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
            if (state_ != State::WAITING_FOR_CONVERGENCE) {
                return;
            }

            nomotion_request_in_flight_ = false;
            try {
                (void)future.get();
                nomotion_update_confirmed_ = true;
            } catch (const std::exception & exception) {
                finish_failure(
                    std::string("AMCL no-motion update failed: ") +
                    exception.what());
            }
        });

    RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        5000,
        "Waiting for fresh, stable AMCL samples from no-motion updates...");
}

void LocalizationBootstrapper::check_map_to_odom_transform()
{
    if (state_ != State::WAITING_FOR_MAP_TO_ODOM_TRANSFORM) {
        return;
    }

    if (localization_timeout_expired()) {
        finish_failure("AMCL reported confidence but map-to-odom TF was unavailable.");
        return;
    }

    if (!tf_buffer_->canTransform(
            config_.goal_frame,
            config_.localization_odom_frame,
            tf2::TimePointZero))
    {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "Waiting for %s -> %s TF after AMCL convergence...",
            config_.goal_frame.c_str(),
            config_.localization_odom_frame.c_str());
        return;
    }

    begin_settling();
}

void LocalizationBootstrapper::begin_settling()
{
    if (state_ != State::WAITING_FOR_MAP_TO_ODOM_TRANSFORM) {
        return;
    }

    cancel_timer(transform_timer_);
    state_ = State::WAITING_FOR_SETTLE;
    settle_started_at_ = std::chrono::steady_clock::now();
    settle_timer_ = node_->create_wall_timer(200ms, [this]() {
        check_settled_localization();
    });
    RCLCPP_INFO(
        node_->get_logger(),
        "AMCL TF is available; keeping the robot stationary for final confirmation.");
}

void LocalizationBootstrapper::check_settled_localization()
{
    if (state_ != State::WAITING_FOR_SETTLE) {
        return;
    }

    if (localization_timeout_expired()) {
        finish_failure("AMCL no-motion localization did not finish final confirmation.");
        return;
    }

    const auto elapsed = std::chrono::steady_clock::now() - settle_started_at_;
    if (elapsed < std::chrono::duration<double>(config_.localization_settle_duration_sec)) {
        return;
    }

    if (!monitor_->is_ready(node_->now())) {
        finish_failure("AMCL localization became stale or lost confidence during final confirmation.");
        return;
    }
    std::string reason;
    if (!amcl_matches_scan_map(reason)) {
        retry_matched_initial_pose(reason);
        return;
    }
    if (!tf_buffer_->canTransform(
            config_.goal_frame,
            config_.localization_odom_frame,
            tf2::TimePointZero))
    {
        finish_failure("map-to-odom TF disappeared during final confirmation.");
        return;
    }

    finish_success();
}

bool LocalizationBootstrapper::amcl_matches_scan_map(std::string & reason) const
{
    if (!config_.localization_scan_match_enabled) {
        return true;
    }
    if (!matched_initial_pose_.has_value()) {
        reason = "scan-map initial pose is unavailable";
        return false;
    }
    const auto amcl_pose = monitor_->latest_amcl_pose();
    const auto base_from_scan = base_from_scan_transform();
    if (!amcl_pose.has_value() || !base_from_scan.has_value()) {
        reason = "AMCL pose or base-to-scan TF is unavailable";
        return false;
    }

    const double position_difference = std::hypot(
        amcl_pose->x - matched_initial_pose_->x,
        amcl_pose->y - matched_initial_pose_->y);
    const double yaw_difference = std::abs(pose_utils::angle_difference(
        amcl_pose->yaw,
        matched_initial_pose_->yaw));
    const auto scan_error = scan_map_matcher_->score_pose(
        PlanarTransform{amcl_pose->x, amcl_pose->y, amcl_pose->yaw},
        *base_from_scan);
    if (!scan_error.has_value()) {
        reason = "scan-map verification data is unavailable";
        return false;
    }
    if (position_difference > config_.localization_scan_match_pose_tolerance_m ||
        yaw_difference > config_.localization_scan_match_yaw_tolerance_rad ||
        *scan_error > config_.localization_scan_match_max_mean_error_m)
    {
        std::ostringstream stream;
        stream << "AMCL diverged from scan-map candidate: position="
               << position_difference << " m, yaw=" << yaw_difference
               << " rad, scan error=" << *scan_error << " m";
        reason = stream.str();
        return false;
    }
    return true;
}

bool LocalizationBootstrapper::retry_matched_initial_pose(const std::string & reason)
{
    if (!matched_initial_pose_.has_value()) {
        finish_failure(reason);
        return false;
    }
    ++matched_initial_pose_retry_count_;
    if (matched_initial_pose_retry_count_ >
        config_.localization_scan_match_max_initial_pose_retries)
    {
        finish_failure(
            "AMCL did not agree with independent scan-map localization: " + reason);
        return false;
    }

    RCLCPP_WARN(
        node_->get_logger(),
        "%s; republishing scan-map initial pose (%d/%d).",
        reason.c_str(),
        matched_initial_pose_retry_count_,
        config_.localization_scan_match_max_initial_pose_retries);
    publish_matched_initial_pose();
    return true;
}

std::optional<PlanarTransform>
LocalizationBootstrapper::base_from_scan_transform() const
{
    const std::string scan_frame = scan_map_matcher_->scan_frame_id();
    if (scan_frame.empty() || !tf_buffer_->canTransform(
            config_.localization_base_frame,
            scan_frame,
            tf2::TimePointZero))
    {
        return std::nullopt;
    }

    try {
        const geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform(
                config_.localization_base_frame,
                scan_frame,
                tf2::TimePointZero);
        return PlanarTransform{
            transform.transform.translation.x,
            transform.transform.translation.y,
            pose_utils::quaternion_to_yaw(transform.transform.rotation)};
    } catch (const std::exception &) {
        return std::nullopt;
    }
}

void LocalizationBootstrapper::finish_success()
{
    if (state_ != State::WAITING_FOR_SETTLE) {
        return;
    }

    state_ = State::SUCCEEDED;
    nomotion_request_in_flight_ = false;
    nomotion_update_confirmed_ = false;
    cancel_timer(global_service_timer_);
    cancel_timer(scan_match_timer_);
    cancel_timer(nomotion_service_timer_);
    cancel_timer(nomotion_update_timer_);
    cancel_timer(transform_timer_);
    cancel_timer(settle_timer_);
    if (config_.localization_scan_match_enabled) {
        RCLCPP_INFO(
            node_->get_logger(),
            "AMCL agrees with scan-map localization; startup motion was not commanded.");
    } else {
        RCLCPP_INFO(
            node_->get_logger(),
            "AMCL no-motion localization completed; startup motion was not commanded.");
    }
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
    nomotion_request_in_flight_ = false;
    nomotion_update_confirmed_ = false;
    cancel_timer(global_service_timer_);
    cancel_timer(scan_match_timer_);
    cancel_timer(nomotion_service_timer_);
    cancel_timer(nomotion_update_timer_);
    cancel_timer(transform_timer_);
    cancel_timer(settle_timer_);
    if (initial_pose_publisher_) {
        initial_pose_publisher_->stop();
    }
    RCLCPP_ERROR(node_->get_logger(), "%s", reason.c_str());
    auto callback = std::move(on_complete_);
    if (callback) {
        callback(false);
    }
}

bool LocalizationBootstrapper::localization_timeout_expired() const
{
    const auto elapsed = std::chrono::steady_clock::now() - started_at_;
    return elapsed > std::chrono::duration<double>(config_.localization_timeout_sec);
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
