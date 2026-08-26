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
    int published_count_{0};
    bool running_{false};
};

}  // namespace auto_patrol
