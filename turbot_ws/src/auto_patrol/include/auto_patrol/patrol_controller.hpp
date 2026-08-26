#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "auto_patrol/patrol_config.hpp"

namespace auto_patrol
{

// 负责 Nav2 Action、waypoint 推进、重试和终态，不负责参数读取或传感器订阅。
// 所有方法和 Action 回调由 Node 的 ROS2 回调线程驱动；状态转换不跨线程暴露。
class PatrolController final
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using ActionClient = rclcpp_action::Client<NavigateToPose>;
    using WrappedResult = GoalHandle::WrappedResult;
    using SendGoalOptions = ActionClient::SendGoalOptions;
    using CompletionCallback = std::function<void(bool success)>;

    PatrolController(
        rclcpp::Node & node,
        PatrolConfig config,
        CompletionCallback on_complete);
    ~PatrolController();

    bool action_server_is_ready() const;
    void start();
    void stop();

private:
    void send_current_goal();
    void retry_or_fail(const std::string & reason);
    void schedule_next_goal(double delay_seconds);
    void finish_success();
    void finish_failure();

    rclcpp::Node * node_;
    PatrolConfig config_;
    CompletionCallback on_complete_;
    ActionClient::SharedPtr action_client_;
    GoalHandle::SharedPtr active_goal_;
    rclcpp::TimerBase::SharedPtr next_goal_timer_;

    std::size_t waypoint_index_{0};
    int retry_count_{0};
    std::uint64_t goal_generation_{0};
    bool started_{false};
    bool stopped_{false};
    bool finished_{false};
};

}  // namespace auto_patrol
