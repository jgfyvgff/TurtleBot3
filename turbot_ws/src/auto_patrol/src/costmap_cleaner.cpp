#include "auto_patrol/costmap_cleaner.hpp"

#include <chrono>
#include <exception>
#include <string>
#include <utility>

using namespace std::chrono_literals;

namespace auto_patrol
{

CostmapCleaner::CostmapCleaner(
    rclcpp::Node & node,
    PatrolConfig config,
    CompletionCallback on_complete)
    : node_(&node),
      config_(std::move(config)),
      on_complete_(std::move(on_complete))
{
    global_costmap_client_ = node_->create_client<ClearEntireCostmap>(
        config_.global_costmap_clear_service);
    local_costmap_client_ = node_->create_client<ClearEntireCostmap>(
        config_.local_costmap_clear_service);
}

CostmapCleaner::~CostmapCleaner()
{
    stop();
}

void CostmapCleaner::start()
{
    if (state_ != State::IDLE) {
        return;
    }

    state_ = State::WAITING_FOR_SERVICES;
    started_at_ = std::chrono::steady_clock::now();
    service_timer_ = node_->create_wall_timer(500ms, [this]() {
        wait_for_services();
    });
    timeout_timer_ = node_->create_wall_timer(100ms, [this]() {
        check_timeout();
    });
}

void CostmapCleaner::stop()
{
    if (state_ == State::STOPPED) {
        return;
    }

    state_ = State::STOPPED;
    cancel_timer(service_timer_);
    cancel_timer(timeout_timer_);
    cancel_timer(refresh_timer_);
    on_complete_ = {};
}

void CostmapCleaner::wait_for_services()
{
    if (state_ != State::WAITING_FOR_SERVICES) {
        return;
    }

    if (!global_costmap_client_->service_is_ready() ||
        !local_costmap_client_->service_is_ready())
    {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "Waiting for Nav2 global and local costmap clear services...");
        return;
    }

    cancel_timer(service_timer_);
    send_clear_requests();
}

void CostmapCleaner::send_clear_requests()
{
    if (state_ != State::WAITING_FOR_SERVICES) {
        return;
    }

    state_ = State::CLEARING;
    pending_request_count_ = 2;
    RCLCPP_INFO(
        node_->get_logger(),
        "Clearing Nav2 global and local costmaps after localization.");
    send_clear_request(global_costmap_client_, config_.global_costmap_clear_service);
    send_clear_request(local_costmap_client_, config_.local_costmap_clear_service);
}

void CostmapCleaner::send_clear_request(
    rclcpp::Client<ClearEntireCostmap>::SharedPtr client,
    const std::string & service_name)
{
    auto request = std::make_shared<ClearEntireCostmap::Request>();
    try {
        client->async_send_request(
            request,
            [this, service_name](
                rclcpp::Client<ClearEntireCostmap>::SharedFuture future) {
                handle_clear_response(service_name, future);
            });
    } catch (const std::exception & exception) {
        finish_failure(
            "Failed to request " + service_name + ": " + exception.what());
    }
}

void CostmapCleaner::handle_clear_response(
    const std::string & service_name,
    rclcpp::Client<ClearEntireCostmap>::SharedFuture future)
{
    if (state_ != State::CLEARING) {
        return;
    }

    try {
        if (!future.get()) {
            finish_failure("Costmap clear service returned no response: " + service_name);
            return;
        }
    } catch (const std::exception & exception) {
        finish_failure(
            "Costmap clear service failed for " + service_name + ": " +
            exception.what());
        return;
    }

    --pending_request_count_;
    if (pending_request_count_ == 0) {
        begin_refresh_wait();
    }
}

void CostmapCleaner::begin_refresh_wait()
{
    if (state_ != State::CLEARING) {
        return;
    }

    state_ = State::WAITING_FOR_COSTMAP_REFRESH;
    cancel_timer(timeout_timer_);
    if (config_.costmap_clear_settle_duration_sec == 0.0) {
        finish_success();
        return;
    }

    const auto refresh_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(config_.costmap_clear_settle_duration_sec));
    refresh_timer_ = node_->create_wall_timer(refresh_duration, [this]() {
        cancel_timer(refresh_timer_);
        finish_success();
    });
}

void CostmapCleaner::check_timeout()
{
    if (state_ != State::WAITING_FOR_SERVICES && state_ != State::CLEARING) {
        return;
    }

    const auto elapsed = std::chrono::steady_clock::now() - started_at_;
    if (elapsed > std::chrono::duration<double>(config_.costmap_clear_timeout_sec)) {
        finish_failure("Timed out while clearing Nav2 costmaps.");
    }
}

void CostmapCleaner::finish_success()
{
    if (state_ != State::WAITING_FOR_COSTMAP_REFRESH) {
        return;
    }

    state_ = State::SUCCEEDED;
    cancel_timer(service_timer_);
    cancel_timer(timeout_timer_);
    cancel_timer(refresh_timer_);
    RCLCPP_INFO(
        node_->get_logger(),
        "Nav2 costmaps are clear and refreshed; starting patrol.");
    auto callback = std::move(on_complete_);
    if (callback) {
        callback(true);
    }
}

void CostmapCleaner::finish_failure(const std::string & reason)
{
    if (state_ == State::FAILED || state_ == State::SUCCEEDED ||
        state_ == State::STOPPED)
    {
        return;
    }

    state_ = State::FAILED;
    cancel_timer(service_timer_);
    cancel_timer(timeout_timer_);
    cancel_timer(refresh_timer_);
    RCLCPP_ERROR(node_->get_logger(), "%s", reason.c_str());
    auto callback = std::move(on_complete_);
    if (callback) {
        callback(false);
    }
}

void CostmapCleaner::cancel_timer(rclcpp::TimerBase::SharedPtr & timer)
{
    if (timer) {
        timer->cancel();
        timer.reset();
    }
}

}  // namespace auto_patrol
