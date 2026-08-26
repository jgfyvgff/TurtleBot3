#include "auto_patrol/patrol_controller.hpp"

#include <utility>

#include "auto_patrol/pose_utils.hpp"

namespace auto_patrol
{

PatrolController::PatrolController(
    rclcpp::Node & node,
    PatrolConfig config,
    CompletionCallback on_complete)
    : node_(&node),
      config_(std::move(config)),
      on_complete_(std::move(on_complete))
{
    action_client_ = rclcpp_action::create_client<NavigateToPose>(
        node_,
        "navigate_to_pose");
}

PatrolController::~PatrolController()
{
    stop();
}

bool PatrolController::action_server_is_ready() const
{
    return action_client_->action_server_is_ready();
}

void PatrolController::start()
{
    if (started_ || stopped_ || finished_) {
        return;
    }
    started_ = true;
    send_current_goal();
}

void PatrolController::stop()
{
    if (stopped_) {
        return;
    }
    stopped_ = true;
    ++goal_generation_;

    if (next_goal_timer_) {
        next_goal_timer_->cancel();
        next_goal_timer_.reset();
    }

    if (active_goal_) {
        action_client_->async_cancel_goal(active_goal_);
        active_goal_.reset();
    }
}

void PatrolController::send_current_goal()
{
    if (stopped_ || finished_) {
        return;
    }

    if (waypoint_index_ >= config_.waypoints.size()) {
        finish_success();
        return;
    }

    if (!action_server_is_ready()) {
        RCLCPP_WARN(
            node_->get_logger(),
            "NavigateToPose server is unavailable; checking again.");
        schedule_next_goal(1.0);
        return;
    }

    const std::size_t current_index = waypoint_index_;
    const Waypoint & waypoint = config_.waypoints[current_index];
    const std::uint64_t generation = ++goal_generation_;

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = config_.goal_frame;
    goal.pose.header.stamp = pose_utils::to_message_time(node_->now());
    goal.pose.pose = pose_utils::make_pose(
        waypoint.x,
        waypoint.y,
        waypoint.yaw);

    RCLCPP_INFO(
        node_->get_logger(),
        "[%zu/3] Sending goal: x=%.3f, y=%.3f, yaw=%.3f rad.",
        current_index + 1,
        waypoint.x,
        waypoint.y,
        waypoint.yaw);

    SendGoalOptions options;
    options.goal_response_callback =
        [this, current_index, generation](GoalHandle::SharedPtr goal_handle) {
            if (stopped_ || finished_ || generation != goal_generation_ ||
                current_index != waypoint_index_)
            {
                return;
            }

            if (!goal_handle) {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "[%zu/3] Goal rejected.",
                    current_index + 1);
                retry_or_fail("goal rejected");
                return;
            }

            active_goal_ = goal_handle;
            RCLCPP_INFO(
                node_->get_logger(),
                "[%zu/3] Goal accepted.",
                current_index + 1);
        };

    options.result_callback =
        [this, current_index, generation](const WrappedResult & result) {
            if (stopped_ || finished_ || generation != goal_generation_ ||
                current_index != waypoint_index_)
            {
                return;
            }

            active_goal_.reset();
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(
                    node_->get_logger(),
                    "[%zu/3] Goal reached.",
                    current_index + 1);
                ++waypoint_index_;
                retry_count_ = 0;
                schedule_next_goal(1.0);
                return;
            }

            if (result.code == rclcpp_action::ResultCode::ABORTED) {
                retry_or_fail("goal aborted");
            } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                retry_or_fail("goal canceled");
            } else {
                retry_or_fail("unknown navigation result");
            }
        };

    action_client_->async_send_goal(goal, options);
}

void PatrolController::retry_or_fail(const std::string & reason)
{
    if (stopped_ || finished_) {
        return;
    }

    active_goal_.reset();
    ++goal_generation_;
    if (retry_count_ < config_.max_retries) {
        ++retry_count_;
        RCLCPP_WARN(
            node_->get_logger(),
            "[%zu/3] %s; retry %d/%d.",
            waypoint_index_ + 1,
            reason.c_str(),
            retry_count_,
            config_.max_retries);
        schedule_next_goal(2.0);
        return;
    }

    RCLCPP_ERROR(
        node_->get_logger(),
        "[%zu/3] %s; retry limit reached.",
        waypoint_index_ + 1,
        reason.c_str());
    finish_failure();
}

void PatrolController::schedule_next_goal(double delay_seconds)
{
    if (stopped_ || finished_) {
        return;
    }
    if (next_goal_timer_) {
        next_goal_timer_->cancel();
        next_goal_timer_.reset();
    }

    next_goal_timer_ = node_->create_wall_timer(
        pose_utils::milliseconds_from_seconds(delay_seconds),
        [this]() {
            if (!next_goal_timer_) {
                return;
            }
            next_goal_timer_->cancel();
            next_goal_timer_.reset();
            send_current_goal();
        });
}

void PatrolController::finish_success()
{
    if (stopped_ || finished_) {
        return;
    }
    finished_ = true;
    RCLCPP_INFO(
        node_->get_logger(),
        "Patrol completed successfully: all %zu goals reached.",
        config_.waypoints.size());
    if (on_complete_) {
        on_complete_(true);
    }
}

void PatrolController::finish_failure()
{
    if (stopped_ || finished_) {
        return;
    }
    finished_ = true;
    RCLCPP_ERROR(
        node_->get_logger(),
        "Patrol failed at waypoint %zu/%zu.",
        waypoint_index_ + 1,
        config_.waypoints.size());
    if (on_complete_) {
        on_complete_(false);
    }
}

}  // namespace auto_patrol
