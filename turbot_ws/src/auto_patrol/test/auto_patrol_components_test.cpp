#include <cmath>
#include <chrono>
#include <limits>
#include <memory>
#include <thread>

#include "gtest/gtest.h"

#include "rclcpp/executors/single_threaded_executor.hpp"

#include "auto_patrol/costmap_cleaner.hpp"
#include "auto_patrol/initial_pose_publisher.hpp"
#include "auto_patrol/localization_bootstrapper.hpp"
#include "auto_patrol/localization_monitor.hpp"
#include "auto_patrol/patrol_config.hpp"
#include "auto_patrol/pose_utils.hpp"
#include "auto_patrol/scan_map_matcher.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

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

auto make_asymmetric_map()
{
    nav_msgs::msg::OccupancyGrid map;
    // 使用 0.1 m 栅格，避免测试中相差 0.2 m 的不同位姿仍落入同一占据单元。
    map.info.resolution = 0.1F;
    map.info.width = 100U;
    map.info.height = 100U;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(
        static_cast<std::size_t>(map.info.width) *
        static_cast<std::size_t>(map.info.height),
        0);

    const auto mark_obstacle = [&map](uint32_t column, uint32_t row) {
        const std::size_t index =
            static_cast<std::size_t>(row) *
            static_cast<std::size_t>(map.info.width) +
            static_cast<std::size_t>(column);
        map.data[index] = 100;
    };
    // 这组不对称障碍端点对应真值位姿 (2.05, 2.05, 0)，用于验证全图匹配的坐标变换。
    mark_obstacle(20U, 5U);
    mark_obstacle(30U, 10U);
    mark_obstacle(40U, 20U);
    mark_obstacle(30U, 30U);
    mark_obstacle(20U, 45U);
    return map;
}

auto make_matching_scan()
{
    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id = "base_footprint";
    scan.angle_min = static_cast<float>(-std::acos(-1.0) / 2.0);
    scan.angle_increment = static_cast<float>(std::acos(-1.0) / 4.0);
    scan.range_min = 0.05F;
    scan.range_max = 4.0F;
    scan.ranges = {
        1.5F,
        static_cast<float>(std::sqrt(2.0)),
        2.0F,
        static_cast<float>(std::sqrt(2.0)),
        2.5F,
    };
    return scan;
}

auto make_repeated_feature_map(bool add_false_candidate_interior_walls)
{
    nav_msgs::msg::OccupancyGrid map;
    map.info.resolution = 0.1F;
    map.info.width = 100U;
    map.info.height = 100U;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(
        static_cast<std::size_t>(map.info.width) *
        static_cast<std::size_t>(map.info.height),
        0);

    const auto mark_obstacle = [&map](uint32_t column, uint32_t row) {
        const std::size_t index =
            static_cast<std::size_t>(row) *
            static_cast<std::size_t>(map.info.width) +
            static_cast<std::size_t>(column);
        map.data[index] = 100;
    };

    // 两个候选位置都具有相同的五个端点特征。仅凭端点距离时，它们是等价解。
    mark_obstacle(20U, 5U);
    mark_obstacle(30U, 10U);
    mark_obstacle(40U, 20U);
    mark_obstacle(30U, 30U);
    mark_obstacle(20U, 45U);
    mark_obstacle(60U, 5U);
    mark_obstacle(70U, 10U);
    mark_obstacle(80U, 20U);
    mark_obstacle(70U, 30U);
    mark_obstacle(60U, 45U);

    if (add_false_candidate_interior_walls) {
        // 第二组端点前额外放置墙体：其端点分数仍为零，但物理激光不可能穿过这些栅格。
        mark_obstacle(60U, 10U);
        mark_obstacle(65U, 15U);
        mark_obstacle(70U, 20U);
        mark_obstacle(65U, 25U);
        mark_obstacle(60U, 30U);
    }
    return map;
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

TEST(PatrolConfigTest, RejectsInvalidNoMotionLocalizationParameters)
{
    auto config = make_valid_config();
    config.localization_global_localization_service.clear();
    EXPECT_THROW(
        auto_patrol::validate_patrol_config(config),
        std::runtime_error);

    config = make_valid_config();
    config.localization_nomotion_update_service.clear();
    EXPECT_THROW(
        auto_patrol::validate_patrol_config(config),
        std::runtime_error);

    config = make_valid_config();
    config.localization_nomotion_update_period_sec = 0.0;
    EXPECT_THROW(
        auto_patrol::validate_patrol_config(config),
        std::runtime_error);

    config = make_valid_config();
    config.localization_odom_frame.clear();
    EXPECT_THROW(
        auto_patrol::validate_patrol_config(config),
        std::runtime_error);

    config = make_valid_config();
    config.localization_settle_duration_sec = 0.0;
    EXPECT_THROW(
        auto_patrol::validate_patrol_config(config),
        std::runtime_error);
}

TEST(PatrolConfigTest, RejectsInvalidScanMatchingParameters)
{
    auto config = make_valid_config();
    config.localization_scan_match_max_beams = 2;
    EXPECT_THROW(auto_patrol::validate_patrol_config(config), std::runtime_error);

    config = make_valid_config();
    config.localization_scan_match_refine_step_m = 0.0;
    EXPECT_THROW(auto_patrol::validate_patrol_config(config), std::runtime_error);

    config = make_valid_config();
    config.localization_scan_match_max_initial_pose_retries = -1;
    EXPECT_THROW(auto_patrol::validate_patrol_config(config), std::runtime_error);
}

TEST(PatrolConfigTest, RejectsInvalidCostmapParameters)
{
    auto config = make_valid_config();
    config.costmap_clear_timeout_sec = 0.0;
    EXPECT_THROW(auto_patrol::validate_patrol_config(config), std::runtime_error);

    config = make_valid_config();
    config.global_costmap_clear_service.clear();
    EXPECT_THROW(auto_patrol::validate_patrol_config(config), std::runtime_error);
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
    EXPECT_FALSE(monitor.is_ready(now));

    auto second = make_pose_message(9, 0.0, 0.0, 0.0);
    second.pose.covariance[0] = 0.01;
    second.pose.covariance[7] = 0.01;
    second.pose.covariance[35] = 0.01;
    monitor.update_amcl_pose(second, now);

    EXPECT_FALSE(monitor.is_ready(now));

    monitor.update_amcl_pose(
        make_pose_message(9, 0.0, 0.0, 0.0), now);
    EXPECT_TRUE(monitor.is_ready(now));
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

    sensor_msgs::msg::LaserScan fresh_scan;
    fresh_scan.header.stamp = make_stamp(12);
    const auto later_time = rclcpp::Time(12, 0, RCL_ROS_TIME);
    monitor.update_scan(fresh_scan, later_time);
    EXPECT_FALSE(monitor.is_ready(later_time));
}

TEST(ScanMapMatcherTest, FindsTheOnlyLowErrorPoseInAnAsymmetricMap)
{
    auto config = make_valid_config();
    config.localization_scan_match_coarse_step_m = 0.5;
    config.localization_scan_match_coarse_yaw_step_rad = std::acos(-1.0) / 4.0;
    config.localization_scan_match_refine_step_m = 0.05;
    config.localization_scan_match_refine_yaw_step_rad = std::acos(-1.0) / 36.0;
    config.localization_scan_match_max_beams = 5;
    config.localization_scan_match_max_range_m = 3.0;
    config.localization_scan_match_max_mean_error_m = 0.05;
    config.localization_scan_match_min_margin_m = 0.05;
    config.localization_scan_match_min_separation_m = 0.5;
    config.localization_scan_match_min_yaw_separation_rad = std::acos(-1.0) / 6.0;

    auto_patrol::ScanMapMatcher matcher(config);
    matcher.update_map(make_asymmetric_map());
    matcher.update_scan(make_matching_scan());

    const auto true_error = matcher.score_pose(
        auto_patrol::PlanarTransform{2.05, 2.05, 0.0},
        auto_patrol::PlanarTransform{});
    const auto displaced_error = matcher.score_pose(
        auto_patrol::PlanarTransform{3.05, 2.05, 0.0},
        auto_patrol::PlanarTransform{});
    ASSERT_TRUE(true_error.has_value());
    ASSERT_TRUE(displaced_error.has_value());
    EXPECT_NEAR(*true_error, 0.0, 1e-9);
    EXPECT_GT(*displaced_error, 0.05);

    const auto result = matcher.find_global_match(auto_patrol::PlanarTransform{});
    ASSERT_TRUE(result.map_and_scan_ready);
    ASSERT_TRUE(result.accepted);
    EXPECT_NEAR(result.best_pose.x, 2.05, 0.10);
    EXPECT_NEAR(result.best_pose.y, 2.05, 0.10);
    EXPECT_NEAR(result.best_pose.yaw, 0.0, 0.10);
}

TEST(ScanMapMatcherTest, UsesFreeSpaceToRejectEndpointOnlyLookalike)
{
    auto config = make_valid_config();
    config.localization_scan_match_coarse_step_m = 0.5;
    config.localization_scan_match_coarse_yaw_step_rad = std::acos(-1.0) / 4.0;
    config.localization_scan_match_refine_step_m = 0.05;
    config.localization_scan_match_refine_yaw_step_rad = std::acos(-1.0) / 36.0;
    config.localization_scan_match_max_beams = 5;
    config.localization_scan_match_max_range_m = 3.0;
    config.localization_scan_match_free_space_max_beams = 5;
    config.localization_scan_match_free_space_penalty_m = 0.25;
    config.localization_scan_match_max_mean_error_m = 0.05;
    config.localization_scan_match_min_margin_m = 0.05;
    config.localization_scan_match_min_separation_m = 0.5;
    config.localization_scan_match_min_yaw_separation_rad = std::acos(-1.0) / 6.0;

    auto_patrol::ScanMapMatcher matcher(config);
    matcher.update_map(make_repeated_feature_map(true));
    matcher.update_scan(make_matching_scan());

    const auto true_error = matcher.score_pose(
        auto_patrol::PlanarTransform{2.05, 2.05, 0.0},
        auto_patrol::PlanarTransform{});
    const auto false_error = matcher.score_pose(
        auto_patrol::PlanarTransform{6.05, 2.05, 0.0},
        auto_patrol::PlanarTransform{});
    ASSERT_TRUE(true_error.has_value());
    ASSERT_TRUE(false_error.has_value());
    EXPECT_NEAR(*true_error, 0.0, 1e-9);
    EXPECT_GE(*false_error, 0.20);

    const auto result = matcher.find_global_match(auto_patrol::PlanarTransform{});
    ASSERT_TRUE(result.map_and_scan_ready);
    EXPECT_TRUE(result.accepted);
    EXPECT_NEAR(result.best_pose.x, 2.05, 0.10);
    EXPECT_NEAR(result.best_pose.y, 2.05, 0.10);
    EXPECT_NEAR(result.best_free_space_mean_penalty_m, 0.0, 1e-9);
}

TEST(ScanMapMatcherTest, RejectsStrictlyIndistinguishableRepeatedFeatures)
{
    auto config = make_valid_config();
    config.localization_scan_match_coarse_step_m = 0.5;
    config.localization_scan_match_coarse_yaw_step_rad = std::acos(-1.0) / 4.0;
    config.localization_scan_match_refine_step_m = 0.05;
    config.localization_scan_match_refine_yaw_step_rad = std::acos(-1.0) / 36.0;
    config.localization_scan_match_max_beams = 5;
    config.localization_scan_match_max_range_m = 3.0;
    config.localization_scan_match_free_space_max_beams = 5;
    config.localization_scan_match_max_mean_error_m = 0.05;
    config.localization_scan_match_min_margin_m = 0.05;
    config.localization_scan_match_min_separation_m = 0.5;
    config.localization_scan_match_min_yaw_separation_rad = std::acos(-1.0) / 6.0;

    auto_patrol::ScanMapMatcher matcher(config);
    matcher.update_map(make_repeated_feature_map(false));
    matcher.update_scan(make_matching_scan());

    const auto result = matcher.find_global_match(auto_patrol::PlanarTransform{});
    ASSERT_TRUE(result.map_and_scan_ready);
    ASSERT_TRUE(result.runner_up_pose.has_value());
    EXPECT_FALSE(result.accepted);
    EXPECT_LT(
        result.runner_up_mean_error_m - result.best_mean_error_m,
        config.localization_scan_match_min_margin_m);
    EXPECT_GE(
        result.runner_up_position_distance_m,
        config.localization_scan_match_min_separation_m);
}

TEST_F(CostmapCleanerTest, BootstrapperUsesScanMatchAndNoMotionServicesWithoutVelocityPublisher)
{
    using Empty = std_srvs::srv::Empty;

    auto config = make_valid_config();
    config.localization_global_localization_service =
        "/test_reinitialize_global_localization";
    config.localization_nomotion_update_service = "/test_request_nomotion_update";
    config.localization_nomotion_update_period_sec = 0.05;
    config.localization_settle_duration_sec = 0.05;
    config.localization_timeout_sec = 2.0;
    config.stable_amcl_samples = 2;
    config.initial_pose_publish_count = 1;
    config.initial_pose_publish_period_sec = 0.05;
    config.initial_pose_wait_sec = 0.05;
    // 使用测试私有话题，避免外部 Gazebo/AMCL 的仿真时间消息污染本测试的系统时间基。
    config.scan_topic = "/test_localization_scan";
    config.amcl_pose_topic = "/test_localization_amcl_pose";

    auto node = std::make_shared<rclcpp::Node>("localization_bootstrapper_test");
    int global_request_count = 0;
    int nomotion_request_count = 0;
    auto global_service = node->create_service<Empty>(
        config.localization_global_localization_service,
        [&global_request_count](const std::shared_ptr<Empty::Request>,
            std::shared_ptr<Empty::Response>) {
            ++global_request_count;
        });
    auto nomotion_service = node->create_service<Empty>(
        config.localization_nomotion_update_service,
        [&nomotion_request_count](const std::shared_ptr<Empty::Request>,
            std::shared_ptr<Empty::Response>) {
            ++nomotion_request_count;
        });

    tf2_ros::StaticTransformBroadcaster static_broadcaster(node);
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = node->now();
    map_to_odom.header.frame_id = config.goal_frame;
    map_to_odom.child_frame_id = config.localization_odom_frame;
    map_to_odom.transform.rotation.w = 1.0;
    static_broadcaster.sendTransform(map_to_odom);

    auto scan_publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(
        config.scan_topic,
        rclcpp::QoS(10));
    auto amcl_publisher = node->create_publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        config.amcl_pose_topic,
        rclcpp::QoS(10));
    auto sensor_timer = node->create_wall_timer(
        std::chrono::milliseconds(20), [&]() {
        auto scan = make_matching_scan();
        scan.header.stamp = node->now();
        scan_publisher->publish(scan);

        geometry_msgs::msg::PoseWithCovarianceStamped amcl_pose;
        amcl_pose.header.stamp = node->now();
        // Fake AMCL 与已知扫描匹配真值一致，验证状态机的默认成功路径。
        amcl_pose.pose.pose = auto_patrol::pose_utils::make_pose(2.05, 2.05, 0.0);
        amcl_publisher->publish(amcl_pose);
    });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    bool completed = false;
    bool success = false;
    {
        auto_patrol::LocalizationMonitor monitor(config);
        auto_patrol::ScanMapMatcher scan_map_matcher(config);
        scan_map_matcher.update_map(make_asymmetric_map());
        auto_patrol::InitialPosePublisher initial_pose_publisher(*node, config);
        // 测试中没有 AutoPatrolNode 负责转发 Topic，因此在这里显式把 Fake 数据送入组件。
        auto scan_subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
            config.scan_topic,
            rclcpp::QoS(10),
            [&monitor, &scan_map_matcher, &node](
                const sensor_msgs::msg::LaserScan::SharedPtr message) {
                monitor.update_scan(*message, node->now());
                scan_map_matcher.update_scan(*message);
            });
        auto amcl_subscription = node->create_subscription<
            geometry_msgs::msg::PoseWithCovarianceStamped>(
            config.amcl_pose_topic,
            rclcpp::QoS(10),
            [&monitor, &node](
                const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
                monitor.update_amcl_pose(*message, node->now());
            });
        auto_patrol::LocalizationBootstrapper bootstrapper(
            *node,
            config,
            monitor,
            scan_map_matcher,
            initial_pose_publisher,
            [&completed, &success](bool result) {
                completed = true;
                success = result;
            });
        bootstrapper.start();

        EXPECT_TRUE(spin_until(
            executor,
            [&completed]() { return completed; },
            // 默认路径包含一次全图匹配、初始位姿发布、服务轮询和 200 ms 静止确认。
            // 3 s 仅是测试执行器调度上限，不改变任何生产超时参数。
            std::chrono::milliseconds(3000)));
        EXPECT_TRUE(success);
        // 默认扫描匹配路径不应先让 AMCL 随机执行全局重定位。
        EXPECT_EQ(global_request_count, 0);
        EXPECT_GE(nomotion_request_count, 1);
        bootstrapper.stop();
    }

    sensor_timer->cancel();
    executor.remove_node(node);
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
