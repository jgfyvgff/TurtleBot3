#pragma once

#include <functional>
#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "auto_patrol/patrol_config.hpp"

namespace auto_patrol
{

// 所有权属于 AutoPatrolNode；该组件负责 /initialpose Publisher 和两个 Timer 的生命周期。
class InitialPosePublisher final
{
public:
    using CompletionCallback = std::function<void()>;

    InitialPosePublisher(rclcpp::Node & node, PatrolConfig config);
    ~InitialPosePublisher();

    void start(CompletionCallback on_complete);
    // 自动扫描匹配得到候选后，通过此重载发布，而不是把候选写回全局参数。
    void start(
        double x,
        double y,
        double yaw,
        CompletionCallback on_complete);
    void stop();

private:
    void publish_once();
    void schedule_wait();
    void complete();

    rclcpp::Node * node_;
    PatrolConfig config_;
    rclcpp::Publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr wait_timer_;
    CompletionCallback on_complete_;
    double target_x_{0.0};
    double target_y_{0.0};
    double target_yaw_{0.0};
    int published_count_{0};
    bool running_{false};
};

}  // namespace auto_patrol
