#include "auto_patrol/initial_pose_publisher.hpp"

#include <utility>

#include "auto_patrol/pose_utils.hpp"

namespace auto_patrol
{

InitialPosePublisher::InitialPosePublisher(
    rclcpp::Node & node,
    PatrolConfig config)
    : node_(&node),
      config_(std::move(config))
{
    publisher_ = node_->create_publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose",
        rclcpp::QoS(10));
}

InitialPosePublisher::~InitialPosePublisher()
{
    stop();
}

void InitialPosePublisher::start(CompletionCallback on_complete)
{
    stop();
    on_complete_ = std::move(on_complete);
    published_count_ = 0;
    running_ = true;

    publish_once();
    if (published_count_ >= config_.initial_pose_publish_count) {
        schedule_wait();
        return;
    }

    publish_timer_ = node_->create_wall_timer(
        pose_utils::milliseconds_from_seconds(
            config_.initial_pose_publish_period_sec),
        [this]() {
            if (!running_) {
                return;
            }
            publish_once();
            if (published_count_ >= config_.initial_pose_publish_count) {
                publish_timer_->cancel();
                publish_timer_.reset();
                schedule_wait();
            }
        });
}

void InitialPosePublisher::stop()
{
    running_ = false;
    if (publish_timer_) {
        publish_timer_->cancel();
        publish_timer_.reset();
    }
    if (wait_timer_) {
        wait_timer_->cancel();
        wait_timer_.reset();
    }
    on_complete_ = {};
}

void InitialPosePublisher::publish_once()
{
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.frame_id = config_.goal_frame;
    message.header.stamp = pose_utils::to_message_time(node_->now());
    message.pose.pose = pose_utils::make_pose(
        config_.initial_pose_x,
        config_.initial_pose_y,
        config_.initial_pose_yaw);
    message.pose.covariance[0] = 0.10;
    message.pose.covariance[7] = 0.10;
    message.pose.covariance[35] = 0.065;
    publisher_->publish(message);

    ++published_count_;
    RCLCPP_INFO(
        node_->get_logger(),
        "Published initial pose %d/%d: x=%.3f, y=%.3f, yaw=%.3f.",
        published_count_,
        config_.initial_pose_publish_count,
        config_.initial_pose_x,
        config_.initial_pose_y,
        config_.initial_pose_yaw);
}

void InitialPosePublisher::schedule_wait()
{
    wait_timer_ = node_->create_wall_timer(
        pose_utils::milliseconds_from_seconds(config_.initial_pose_wait_sec),
        [this]() {
            if (!running_) {
                return;
            }
            wait_timer_->cancel();
            wait_timer_.reset();
            complete();
        });
}

void InitialPosePublisher::complete()
{
    running_ = false;
    auto callback = std::move(on_complete_);
    if (callback) {
        callback();
    }
}

}  // namespace auto_patrol
