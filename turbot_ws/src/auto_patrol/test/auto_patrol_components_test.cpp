#include <cmath>
#include <chrono>
#include <limits>
#include <memory>
#include <thread>

#include "gtest/gtest.h"

#include "rclcpp/executors/single_threaded_executor.hpp"

#include "auto_patrol/costmap_cleaner.hpp"
#include "auto_patrol/localization_monitor.hpp"
#include "auto_patrol/localization_safety.hpp"
#include "auto_patrol/patrol_config.hpp"
#include "auto_patrol/pose_utils.hpp"

namespace
{

auto make_stamp(int32_t seconds)
{
    builtin_interfaces::msg::Time stamp;
    stamp.sec = seconds;
    stamp.nanosec = 0U;
    return stamp;
}

auto make_pose_message(int32_t seconds, double x, double y, double yaw)
{
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.stamp = make_stamp(seconds);
    message.pose.pose = auto_patrol::pose_utils::make_pose(x, y, yaw);
    return message;
}

auto make_scan_message(
    int32_t seconds,
    const std::vector<float> & ranges)
{
    sensor_msgs::msg::LaserScan message;
    message.header.stamp = make_stamp(seconds);
    message.angle_min = -0.4F;
    message.angle_increment = 0.2F;
    message.range_min = 0.05F;
    message.range_max = 3.5F;
    message.ranges = ranges;
    return message;
}

auto make_valid_config()
{
    auto_patrol::PatrolConfig config;
    config.waypoints = {
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {1.0, 1.0, 1.57},
    };
    return config;
}

class CostmapCleanerTest : public testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    static void TearDownTestSuite()
    {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }
};

template<typename Predicate>
bool spin_until(
    rclcpp::executors::SingleThreadedExecutor & executor,
    Predicate predicate,
    std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some(std::chrono::milliseconds(10));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return predicate();
}

}  // namespace

TEST(PatrolConfigTest, AcceptsValidConfiguration)
{
    EXPECT_NO_THROW(auto_patrol::validate_patrol_config(make_valid_config()));
}

TEST(PatrolConfigTest, RejectsInvalidRetryCount)
{
    auto config = make_valid_config();
    config.max_retries = -1;
    EXPECT_THROW(
        auto_patrol::validate_patrol_config(config),
        std::runtime_error);
}

TEST(PatrolConfigTest, RejectsNonFiniteWaypoint)
{
    auto config = make_valid_config();
    config.waypoints[1].x = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(
        auto_patrol::validate_patrol_config(config),
        std::runtime_error);
}

TEST(PatrolConfigTest, RejectsConfigurationWithoutLocalizationMode)
{
    auto config = make_valid_config();
    config.automatic_global_localization = false;
    config.use_manual_initial_pose_fallback = false;
    EXPECT_THROW(
        auto_patrol::validate_patrol_config(config),
        std::runtime_error);
}

TEST(PatrolConfigTest, RejectsBothLocalizationModes)
{
    auto config = make_valid_config();
    config.automatic_global_localization = true;
    config.use_manual_initial_pose_fallback = true;
    EXPECT_THROW(
        auto_patrol::validate_patrol_config(config),
        std::runtime_error);
}

TEST(PatrolConfigTest, RejectsInvalidLocalizationExploration)
{
    auto config = make_valid_config();
    config.localization_cmd_vel_topic.clear();
    EXPECT_THROW(
        auto_patrol::validate_patrol_config(config),
        std::runtime_error);

    config = make_valid_config();
    config.localization_exploration_angular_speed = 0.0;
    EXPECT_THROW(
        auto_patrol::validate_patrol_config(config),
        std::runtime_error);

    config = make_valid_config();
    config.localization_exploration_max_duration_sec = -1.0;
    EXPECT_THROW(
        auto_patrol::validate_patrol_config(config),
        std::runtime_error);

    config = make_valid_config();
    config.localization_settle_duration_sec = 0.0;
    EXPECT_THROW(
        auto_patrol::validate_patrol_config(config),
        std::runtime_error);
}

TEST(PatrolConfigTest, RejectsInvalidTranslationAndCostmapParameters)
{
    auto config = make_valid_config();
    config.localization_exploration_linear_speed = 0.0;
    EXPECT_THROW(auto_patrol::validate_patrol_config(config), std::runtime_error);

    config = make_valid_config();
    config.localization_exploration_translation_distance = -0.1;
    EXPECT_THROW(auto_patrol::validate_patrol_config(config), std::runtime_error);

    config = make_valid_config();
    config.localization_exploration_front_sector_half_angle = 4.0;
    EXPECT_THROW(auto_patrol::validate_patrol_config(config), std::runtime_error);

    config = make_valid_config();
    config.costmap_clear_timeout_sec = 0.0;
    EXPECT_THROW(auto_patrol::validate_patrol_config(config), std::runtime_error);

    config = make_valid_config();
    config.global_costmap_clear_service.clear();
    EXPECT_THROW(auto_patrol::validate_patrol_config(config), std::runtime_error);
}

TEST(LocalizationSafetyTest, AllowsTranslationWithValidFrontClearance)
{
    const auto scan = make_scan_message(9, {1.0F, 0.8F, 0.7F, 0.8F, 1.0F});
    EXPECT_TRUE(auto_patrol::localization_safety::scan_has_safe_front_clearance(
        scan,
        0.45,
        0.35));
    EXPECT_NEAR(
        auto_patrol::localization_safety::planar_distance(0.0, 0.0, 0.18, 0.24),
        0.30,
        1e-9);
}

TEST(LocalizationSafetyTest, RequiresClearanceForTheWholeTranslation)
{
    const auto scan = make_scan_message(9, {1.0F, 0.8F, 0.69F, 0.8F, 1.0F});

    EXPECT_TRUE(auto_patrol::localization_safety::scan_has_safe_front_clearance(
        scan, 0.45, 0.35));
    EXPECT_FALSE(auto_patrol::localization_safety::scan_has_safe_front_clearance(
        scan, 0.70, 0.35));
}

TEST(LocalizationSafetyTest, RejectsObstacleAndInvalidFrontMeasurements)
{
    auto blocked = make_scan_message(9, {1.0F, 0.8F, 0.40F, 0.8F, 1.0F});
    EXPECT_FALSE(auto_patrol::localization_safety::scan_has_safe_front_clearance(
        blocked,
        0.45,
        0.35));

    const float nan = std::numeric_limits<float>::quiet_NaN();
    auto invalid = make_scan_message(9, {nan, nan, nan, nan, nan});
    EXPECT_FALSE(auto_patrol::localization_safety::scan_has_safe_front_clearance(
        invalid,
        0.45,
        0.35));
}

TEST(PoseUtilsTest, NormalizesAnglesAndComputesDifference)
{
    constexpr double pi = 3.14159265358979323846;
    EXPECT_NEAR(auto_patrol::pose_utils::normalize_angle(3.0 * pi), pi, 1e-9);
    EXPECT_NEAR(
        auto_patrol::pose_utils::angle_difference(-pi + 0.1, pi - 0.1),
        0.2,
        1e-9);
}

TEST(PoseUtilsTest, ConvertsYawToQuaternionAndBack)
{
    constexpr double yaw = 1.2;
    const auto pose = auto_patrol::pose_utils::make_pose(1.0, 2.0, yaw);
    EXPECT_NEAR(
        auto_patrol::pose_utils::quaternion_to_yaw(pose.orientation),
        yaw,
        1e-9);
}

TEST(PoseUtilsTest, RejectsStaleAndFutureTimestamps)
{
    const auto now = rclcpp::Time(10, 0, RCL_ROS_TIME);
    EXPECT_TRUE(auto_patrol::pose_utils::timestamp_is_current(
        make_stamp(9), now, 2.0, 0.5));
    EXPECT_FALSE(auto_patrol::pose_utils::timestamp_is_current(
        make_stamp(7), now, 2.0, 0.5));
    EXPECT_FALSE(auto_patrol::pose_utils::timestamp_is_current(
        make_stamp(11), now, 2.0, 0.5));
    EXPECT_FALSE(auto_patrol::pose_utils::timestamp_is_current(
        make_stamp(0), now, 2.0, 0.5));
}

TEST(LocalizationMonitorTest, RequiresStableAmclSamples)
{
    auto config = make_valid_config();
    config.stable_amcl_samples = 3;
    auto_patrol::LocalizationMonitor monitor(config);
    const auto now = rclcpp::Time(10, 0, RCL_ROS_TIME);

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = make_stamp(9);
    EXPECT_TRUE(monitor.update_scan(scan, now));
    EXPECT_FALSE(monitor.is_ready(now));

    EXPECT_TRUE(monitor.update_amcl_pose(
        make_pose_message(9, 0.0, 0.0, 0.0), now));
    EXPECT_FALSE(monitor.is_ready(now));
    EXPECT_TRUE(monitor.update_amcl_pose(
        make_pose_message(9, 0.01, 0.01, 0.01), now));
    EXPECT_FALSE(monitor.is_ready(now));
    EXPECT_TRUE(monitor.update_amcl_pose(
        make_pose_message(9, 0.02, 0.01, 0.02), now));
    EXPECT_TRUE(monitor.is_ready(now));
}

TEST(LocalizationMonitorTest, StaleAmclClearsStableState)
{
    auto config = make_valid_config();
    config.stable_amcl_samples = 2;
    auto_patrol::LocalizationMonitor monitor(config);
    const auto now = rclcpp::Time(10, 0, RCL_ROS_TIME);

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = make_stamp(9);
    monitor.update_scan(scan, now);
    monitor.update_amcl_pose(make_pose_message(9, 0.0, 0.0, 0.0), now);
    monitor.update_amcl_pose(make_pose_message(9, 0.0, 0.0, 0.0), now);
    EXPECT_TRUE(monitor.is_ready(now));

    EXPECT_FALSE(monitor.update_amcl_pose(
        make_pose_message(1, 0.0, 0.0, 0.0), now));
    EXPECT_FALSE(monitor.is_ready(now));
}

TEST(LocalizationMonitorTest, RequiresLowAmclCovariance)
{
    auto config = make_valid_config();
    config.stable_amcl_samples = 2;
    config.localization_position_variance_threshold = 0.1;
    config.localization_yaw_variance_threshold = 0.05;
    auto_patrol::LocalizationMonitor monitor(config);
    const auto now = rclcpp::Time(10, 0, RCL_ROS_TIME);

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = make_stamp(9);
    monitor.update_scan(scan, now);

    auto first = make_pose_message(9, 0.0, 0.0, 0.0);
    first.pose.covariance[0] = 0.2;
    first.pose.covariance[7] = 0.01;
    first.pose.covariance[35] = 0.01;
    monitor.update_amcl_pose(first, now);
    EXPECT_FALSE(monitor.confidence_is_sufficient(now));

    auto second = make_pose_message(9, 0.0, 0.0, 0.0);
    second.pose.covariance[0] = 0.01;
    second.pose.covariance[7] = 0.01;
    second.pose.covariance[35] = 0.01;
    monitor.update_amcl_pose(second, now);

    EXPECT_FALSE(monitor.is_ready(now));
    EXPECT_TRUE(monitor.confidence_is_sufficient(now));
}

TEST(LocalizationMonitorTest, ResetRequiresNewLocalizationSamples)
{
    auto config = make_valid_config();
    config.stable_amcl_samples = 2;
    auto_patrol::LocalizationMonitor monitor(config);
    const auto now = rclcpp::Time(10, 0, RCL_ROS_TIME);

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = make_stamp(9);
    monitor.update_scan(scan, now);
    monitor.update_amcl_pose(make_pose_message(9, 0.0, 0.0, 0.0), now);
    monitor.update_amcl_pose(make_pose_message(9, 0.0, 0.0, 0.0), now);
    ASSERT_TRUE(monitor.is_ready(now));

    monitor.reset_for_relocalization();
    EXPECT_FALSE(monitor.is_ready(now));

    monitor.update_scan(scan, now);
    monitor.update_amcl_pose(make_pose_message(9, 0.0, 0.0, 0.0), now);
    EXPECT_FALSE(monitor.is_ready(now));
    monitor.update_amcl_pose(make_pose_message(9, 0.0, 0.0, 0.0), now);
    EXPECT_TRUE(monitor.is_ready(now));
}

TEST(LocalizationMonitorTest, RequiresCurrentScanForTranslationClearance)
{
    auto config = make_valid_config();
    auto_patrol::LocalizationMonitor monitor(config);
    const auto now = rclcpp::Time(10, 0, RCL_ROS_TIME);
    const auto scan = make_scan_message(9, {1.0F, 0.8F, 0.7F, 0.8F, 1.0F});

    ASSERT_TRUE(monitor.update_scan(scan, now));
    EXPECT_TRUE(monitor.front_clearance_is_safe(now, 0.45, 0.35));

    const auto stale_now = rclcpp::Time(12, 0, RCL_ROS_TIME);
    EXPECT_FALSE(monitor.front_clearance_is_safe(stale_now, 0.45, 0.35));
}

TEST(LocalizationMonitorTest, RequiresCurrentAmclPose)
{
    auto config = make_valid_config();
    config.stable_amcl_samples = 2;
    auto_patrol::LocalizationMonitor monitor(config);

    const auto current_time = rclcpp::Time(10, 0, RCL_ROS_TIME);
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = make_stamp(9);
    monitor.update_scan(scan, current_time);
    monitor.update_amcl_pose(
        make_pose_message(9, 0.0, 0.0, 0.0),
        current_time);
    monitor.update_amcl_pose(
        make_pose_message(9, 0.0, 0.0, 0.0),
        current_time);
    ASSERT_TRUE(monitor.is_ready(current_time));
    ASSERT_TRUE(monitor.relocalization_is_ready(current_time));

    sensor_msgs::msg::LaserScan fresh_scan;
    fresh_scan.header.stamp = make_stamp(12);
    const auto later_time = rclcpp::Time(12, 0, RCL_ROS_TIME);
    monitor.update_scan(fresh_scan, later_time);
    EXPECT_FALSE(monitor.is_ready(later_time));
    EXPECT_FALSE(monitor.relocalization_is_ready(later_time));
    EXPECT_FALSE(monitor.confidence_is_sufficient(later_time));

}

TEST(LocalizationMonitorTest, RelocalizationAcceptsExpectedYawMotion)
{
    auto config = make_valid_config();
    config.stable_amcl_samples = 3;
    auto_patrol::LocalizationMonitor monitor(config);
    const auto now = rclcpp::Time(10, 0, RCL_ROS_TIME);

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = make_stamp(9);
    monitor.reset_for_relocalization();
    monitor.update_scan(scan, now);

    monitor.update_amcl_pose(make_pose_message(9, 0.0, 0.0, 0.0), now);
    monitor.update_amcl_pose(make_pose_message(9, 0.0, 0.0, 0.25), now);
    EXPECT_FALSE(monitor.is_ready(now));
    EXPECT_FALSE(monitor.relocalization_is_ready(now));

    monitor.update_amcl_pose(make_pose_message(9, 0.0, 0.0, 0.50), now);
    EXPECT_FALSE(monitor.is_ready(now));
    EXPECT_TRUE(monitor.relocalization_is_ready(now));
}

TEST(LocalizationMonitorTest, TranslationValidationKeepsScanButRequiresNewAmclEvidence)
{
    auto config = make_valid_config();
    config.stable_amcl_samples = 2;
    auto_patrol::LocalizationMonitor monitor(config);
    const auto now = rclcpp::Time(10, 0, RCL_ROS_TIME);
    const auto scan = make_scan_message(9, {1.0F, 0.8F, 0.7F, 0.8F, 1.0F});

    monitor.update_scan(scan, now);
    monitor.update_amcl_pose(make_pose_message(9, 0.0, 0.0, 0.0), now);
    monitor.update_amcl_pose(make_pose_message(9, 0.0, 0.0, 0.1), now);
    ASSERT_TRUE(monitor.relocalization_is_ready(now));

    monitor.begin_translation_validation();
    EXPECT_TRUE(monitor.front_clearance_is_safe(now, 0.45, 0.35));
    EXPECT_FALSE(monitor.relocalization_is_ready(now));

    monitor.update_amcl_pose(make_pose_message(9, 0.25, 0.0, 0.2), now);
    monitor.update_amcl_pose(make_pose_message(9, 0.25, 0.0, 0.4), now);
    EXPECT_TRUE(monitor.relocalization_is_ready(now));
    EXPECT_TRUE(monitor.confidence_is_sufficient(now));
}

TEST_F(CostmapCleanerTest, CompletesWhenBothFakeServicesRespond)
{
    using ClearEntireCostmap = nav2_msgs::srv::ClearEntireCostmap;

    auto config = make_valid_config();
    config.global_costmap_clear_service = "/test_global_costmap_clear";
    config.local_costmap_clear_service = "/test_local_costmap_clear";
    config.costmap_clear_timeout_sec = 2.0;
    config.costmap_clear_settle_duration_sec = 0.0;
    auto node = std::make_shared<rclcpp::Node>("costmap_cleaner_success_test");
    auto global_service = node->create_service<ClearEntireCostmap>(
        config.global_costmap_clear_service,
        [](const std::shared_ptr<ClearEntireCostmap::Request>,
            std::shared_ptr<ClearEntireCostmap::Response>) {});
    auto local_service = node->create_service<ClearEntireCostmap>(
        config.local_costmap_clear_service,
        [](const std::shared_ptr<ClearEntireCostmap::Request>,
            std::shared_ptr<ClearEntireCostmap::Response>) {});
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    bool completed = false;
    bool success = false;
    {
        auto_patrol::CostmapCleaner cleaner(
            *node,
            config,
            [&completed, &success](bool result) {
                completed = true;
                success = result;
            });
        cleaner.start();
        EXPECT_TRUE(spin_until(
            executor,
            [&completed]() { return completed; },
            std::chrono::milliseconds(1500)));
        EXPECT_TRUE(success);
        cleaner.stop();
    }

    executor.remove_node(node);
}

TEST_F(CostmapCleanerTest, FailsWhenClearServicesAreUnavailable)
{
    auto config = make_valid_config();
    config.global_costmap_clear_service = "/missing_global_costmap_clear";
    config.local_costmap_clear_service = "/missing_local_costmap_clear";
    config.costmap_clear_timeout_sec = 0.15;
    auto node = std::make_shared<rclcpp::Node>("costmap_cleaner_failure_test");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    bool completed = false;
    bool success = true;
    {
        auto_patrol::CostmapCleaner cleaner(
            *node,
            config,
            [&completed, &success](bool result) {
                completed = true;
                success = result;
            });
        cleaner.start();
        EXPECT_TRUE(spin_until(
            executor,
            [&completed]() { return completed; },
            std::chrono::milliseconds(1000)));
        EXPECT_FALSE(success);
        cleaner.stop();
    }

    executor.remove_node(node);
}
