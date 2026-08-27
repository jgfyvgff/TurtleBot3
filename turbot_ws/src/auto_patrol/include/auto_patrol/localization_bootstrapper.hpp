#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "auto_patrol/initial_pose_publisher.hpp"
#include "auto_patrol/localization_monitor.hpp"
#include "auto_patrol/patrol_config.hpp"
#include "auto_patrol/scan_map_matcher.hpp"

namespace auto_patrol
{

// 启用扫描匹配时，该组件先从静态地图与当前 LaserScan 推导全局初始位姿，再交由
// AMCL 的无运动更新进行确认；关闭扫描匹配时才回退为 AMCL 全局重定位服务。
// 无论哪条路径，该组件从不发布 Twist，也不控制机器人位姿。
class LocalizationBootstrapper final
{
public:
    using CompletionCallback = std::function<void(bool success)>;

    LocalizationBootstrapper(
        rclcpp::Node & node,
        PatrolConfig config,
        LocalizationMonitor & monitor,
        ScanMapMatcher & scan_map_matcher,
        InitialPosePublisher & initial_pose_publisher,
        CompletionCallback on_complete);
    ~LocalizationBootstrapper();

    void start();
    void stop();

private:
    // Timer 驱动所有等待和状态转换，避免在 ROS2 服务回调中阻塞执行器。
    enum class State
    {
        IDLE,
        WAITING_FOR_GLOBAL_LOCALIZATION_SERVICE,
        REQUESTING_GLOBAL_LOCALIZATION,
        WAITING_FOR_SCAN_MATCH,
        PUBLISHING_MATCHED_INITIAL_POSE,
        WAITING_FOR_NOMOTION_UPDATE_SERVICE,
        WAITING_FOR_CONVERGENCE,
        WAITING_FOR_MAP_TO_ODOM_TRANSFORM,
        WAITING_FOR_SETTLE,
        SUCCEEDED,
        FAILED,
        STOPPED
    };

    void wait_for_global_localization_service();
    void request_global_localization();
    void wait_for_scan_match();
    void publish_matched_initial_pose();
    void begin_nomotion_updates();
    void wait_for_nomotion_update_service();
    void no_motion_update_tick();
    void check_map_to_odom_transform();
    void begin_settling();
    void check_settled_localization();
    bool amcl_matches_scan_map(std::string & reason) const;
    bool retry_matched_initial_pose(const std::string & reason);
    std::optional<PlanarTransform> base_from_scan_transform() const;
    void finish_success();
    void finish_failure(const std::string & reason);
    bool localization_timeout_expired() const;
    void cancel_timer(rclcpp::TimerBase::SharedPtr & timer);

    rclcpp::Node * node_;
    PatrolConfig config_;
    LocalizationMonitor * monitor_;
    ScanMapMatcher * scan_map_matcher_;
    InitialPosePublisher * initial_pose_publisher_;
    CompletionCallback on_complete_;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr global_localization_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr nomotion_update_client_;
    // Buffer 与 Listener 仅观察 TF；不创建独立线程，由 Node 的执行器处理订阅回调。
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr global_service_timer_;
    rclcpp::TimerBase::SharedPtr scan_match_timer_;
    rclcpp::TimerBase::SharedPtr nomotion_service_timer_;
    rclcpp::TimerBase::SharedPtr nomotion_update_timer_;
    rclcpp::TimerBase::SharedPtr transform_timer_;
    rclcpp::TimerBase::SharedPtr settle_timer_;

    State state_{State::IDLE};
    bool nomotion_request_in_flight_{false};
    // 只有全局重定位之后至少一次服务响应成功，AMCL 样本才属于本轮无运动定位证据。
    bool nomotion_update_confirmed_{false};
    std::optional<PlanarTransform> matched_initial_pose_;
    int matched_initial_pose_retry_count_{0};
    std::chrono::steady_clock::time_point started_at_;
    std::chrono::steady_clock::time_point settle_started_at_;
};

}  // namespace auto_patrol
