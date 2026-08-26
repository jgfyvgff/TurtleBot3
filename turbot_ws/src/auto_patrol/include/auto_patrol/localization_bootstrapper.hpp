#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include "auto_patrol/localization_monitor.hpp"
#include "auto_patrol/patrol_config.hpp"

namespace auto_patrol
{

// 通过 AMCL 全局重定位服务启动定位，不要求调用方预先提供 x、y、yaw。
// 该组件只负责服务请求和收敛状态机，传感器状态由 LocalizationMonitor 提供。
class LocalizationBootstrapper final
{
public:
    using CompletionCallback = std::function<void(bool success)>;

    LocalizationBootstrapper(
        rclcpp::Node & node,
        PatrolConfig config,
        LocalizationMonitor & monitor,
        CompletionCallback on_complete);
    ~LocalizationBootstrapper();

    void start();
    void stop();
    // Node 的 /odom 回调只更新最近样本；状态机由 Timer 推进，回调内不阻塞。
    void update_odometry(
        const nav_msgs::msg::Odometry & message,
        const rclcpp::Time & now);

private:
    enum class State
    {
        IDLE,
        WAITING_FOR_GLOBAL_LOCALIZATION_SERVICE,
        REQUESTING_GLOBAL_LOCALIZATION,
        // 此状态允许机器人主动旋转，因此不要求相邻 AMCL yaw 保持不变。
        EXPLORING,
        WAITING_FOR_CONVERGENCE,
        // 首次低协方差只代表候选位置，先寻找有足够前方净空的朝向。
        WAITING_FOR_TRANSLATION_CANDIDATE,
        // 使用 odom 测量实际平移距离，禁止按速度乘时间估算。
        TRANSLATING_FOR_VALIDATION,
        // 平移开始前会清除旧 AMCL 证据；本状态收集平移及后续安全旋转产生的新样本。
        WAITING_FOR_SECOND_LOCALIZATION,
        // 已获得验证周期的低协方差证据；静止后只确认结果仍然新鲜。
        WAITING_FOR_SETTLE,
        SUCCEEDED,
        FAILED,
        STOPPED
    };

    void wait_for_service();
    void request_global_localization();
    void check_convergence();
    void exploration_tick();
    void begin_translation_candidate();
    void translation_candidate_tick();
    void start_translation();
    void translation_tick();
    void resume_translation_candidate();
    void begin_second_localization();
    void check_second_localization();
    void begin_settling();
    void check_settled_localization();
    void finish_success();
    void finish_failure(const std::string & reason);
    void publish_angular_velocity();
    void publish_linear_velocity();
    void publish_zero_velocity();
    bool localization_timeout_expired() const;
    bool odometry_is_current(const rclcpp::Time & now) const;
    void cancel_timer(rclcpp::TimerBase::SharedPtr & timer);

    struct OdometrySample
    {
        double x;
        double y;
        builtin_interfaces::msg::Time stamp;
    };

    rclcpp::Node * node_;
    PatrolConfig config_;
    LocalizationMonitor * monitor_;
    CompletionCallback on_complete_;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr
        global_localization_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
        localization_cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr service_timer_;
    rclcpp::TimerBase::SharedPtr convergence_timer_;
    rclcpp::TimerBase::SharedPtr exploration_timer_;
    rclcpp::TimerBase::SharedPtr translation_candidate_timer_;
    rclcpp::TimerBase::SharedPtr translation_timer_;
    rclcpp::TimerBase::SharedPtr second_localization_timer_;
    rclcpp::TimerBase::SharedPtr settle_timer_;

    State state_{State::IDLE};
    std::chrono::steady_clock::time_point started_at_;
    std::chrono::steady_clock::time_point exploration_started_at_;
    std::chrono::steady_clock::time_point translation_started_at_;
    std::chrono::steady_clock::time_point settle_started_at_;
    std::optional<OdometrySample> latest_odometry_;
    // 未进入平移状态前不存在起点，optional 避免伪造一个无时间戳的 odom 样本。
    std::optional<OdometrySample> translation_start_;
};

}  // namespace auto_patrol
