#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
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

private:
    enum class State
    {
        IDLE,
        WAITING_FOR_GLOBAL_LOCALIZATION_SERVICE,
        REQUESTING_GLOBAL_LOCALIZATION,
        // 此状态允许机器人主动旋转，因此不要求相邻 AMCL yaw 保持不变。
        EXPLORING,
        WAITING_FOR_CONVERGENCE,
        // 已发布零速度，等待 AMCL 消化最后一次旋转时的扫描。
        WAITING_FOR_SETTLE,
        SUCCEEDED,
        FAILED,
        STOPPED
    };

    void wait_for_service();
    void request_global_localization();
    void check_convergence();
    void exploration_tick();
    void begin_settling();
    void check_settled_localization();
    void finish_success();
    void finish_failure(const std::string & reason);
    void publish_zero_velocity();
    void cancel_timer(rclcpp::TimerBase::SharedPtr & timer);

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
    rclcpp::TimerBase::SharedPtr settle_timer_;

    State state_{State::IDLE};
    std::chrono::steady_clock::time_point started_at_;
    std::chrono::steady_clock::time_point exploration_started_at_;
    std::chrono::steady_clock::time_point settle_started_at_;
};

}  // namespace auto_patrol
