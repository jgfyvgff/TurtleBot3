#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "rclcpp/rclcpp.hpp"

#include "auto_patrol/patrol_config.hpp"

namespace auto_patrol
{

// 在定位结束后清理 Nav2 的全局和局部代价地图，避免重定位阶段遗留的观测
// 直接影响第一个导航目标。对象由 AutoPatrolNode 持有；所有回调均由同一
// ROS2 executor 驱动，因此状态机不跨线程共享，stop() 仍会取消全部 Timer。
class CostmapCleaner final
{
public:
    using ClearEntireCostmap = nav2_msgs::srv::ClearEntireCostmap;
    using CompletionCallback = std::function<void(bool success)>;

    CostmapCleaner(
        rclcpp::Node & node,
        PatrolConfig config,
        CompletionCallback on_complete);
    ~CostmapCleaner();

    // IDLE 状态只能启动一次；成功、失败或停止后不会重复发送清理请求。
    void start();
    // 可重复调用。停止后不再推进异步服务回调，也不会触发完成回调。
    void stop();

private:
    enum class State
    {
        IDLE,
        WAITING_FOR_SERVICES,
        CLEARING,
        WAITING_FOR_COSTMAP_REFRESH,
        SUCCEEDED,
        FAILED,
        STOPPED
    };

    void wait_for_services();
    void send_clear_requests();
    void send_clear_request(
        rclcpp::Client<ClearEntireCostmap>::SharedPtr client,
        const std::string & service_name);
    void handle_clear_response(
        const std::string & service_name,
        rclcpp::Client<ClearEntireCostmap>::SharedFuture future);
    void begin_refresh_wait();
    void check_timeout();
    void finish_success();
    void finish_failure(const std::string & reason);
    void cancel_timer(rclcpp::TimerBase::SharedPtr & timer);

    rclcpp::Node * node_;
    PatrolConfig config_;
    CompletionCallback on_complete_;
    rclcpp::Client<ClearEntireCostmap>::SharedPtr global_costmap_client_;
    rclcpp::Client<ClearEntireCostmap>::SharedPtr local_costmap_client_;
    rclcpp::TimerBase::SharedPtr service_timer_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    rclcpp::TimerBase::SharedPtr refresh_timer_;

    State state_{State::IDLE};
    std::chrono::steady_clock::time_point started_at_;
    int pending_request_count_{0};
};

}  // namespace auto_patrol
