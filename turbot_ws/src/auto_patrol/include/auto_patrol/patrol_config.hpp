#pragma once

#include <string>
#include <vector>

#include "rclcpp/node.hpp"

namespace auto_patrol
{

struct Waypoint
{
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

// 集中保存运行参数，避免导航流程中散落重复的魔法值。
struct PatrolConfig
{
    std::string goal_frame{"map"};
    std::string amcl_pose_topic{"/amcl_pose"};
    std::string scan_topic{"/scan"};
    std::string odom_topic{"/odom"};

    // 默认让 AMCL 在地图上全局初始化粒子，不要求预先知道机器人坐标。
    bool automatic_global_localization{true};
    double localization_timeout_sec{120.0};
    double localization_position_variance_threshold{0.25};
    double localization_yaw_variance_threshold{0.10};
    // 定位阶段通过 Nav2 的速度输入通道低速旋转，让 AMCL 获得新的扫描观测。
    bool localization_exploration_enabled{true};
    std::string localization_cmd_vel_topic{"/cmd_vel_nav"};
    double localization_exploration_angular_speed{0.20};
    double localization_exploration_max_duration_sec{40.0};
    // 主动验证使用低速短距离平移；单位依次为 m/s、m、s、m 和 rad。
    double localization_exploration_linear_speed{0.06};
    double localization_exploration_translation_distance{0.25};
    double localization_exploration_translation_timeout_sec{8.0};
    double localization_exploration_min_front_clearance{0.45};
    double localization_exploration_front_sector_half_angle{0.35};
    // 探索停止后留给 AMCL 处理最后一帧扫描的确认时间，单位为 s。
    double localization_settle_duration_sec{1.0};

    // 清理服务完成后等待 costmap 重新融合当前扫描，避免刚清空的瞬间发送目标。
    double costmap_clear_timeout_sec{5.0};
    double costmap_clear_settle_duration_sec{2.0};
    std::string global_costmap_clear_service{
        "/global_costmap/clear_entirely_global_costmap"};
    std::string local_costmap_clear_service{
        "/local_costmap/clear_entirely_local_costmap"};

    // 仅用于调试或已知出生点的兼容模式，不是默认定位流程。
    bool use_manual_initial_pose_fallback{false};
    int max_retries{1};
    int stable_amcl_samples{5};

    double stable_position_tolerance{0.05};
    double stable_yaw_tolerance{0.10};
    double max_message_age_sec{2.0};
    double future_message_tolerance_sec{0.5};

    double initial_pose_x{0.0};
    double initial_pose_y{0.0};
    double initial_pose_yaw{0.0};
    int initial_pose_publish_count{5};
    double initial_pose_publish_period_sec{1.0};
    double initial_pose_wait_sec{3.0};

    std::vector<Waypoint> waypoints;
};

void declare_patrol_parameters(rclcpp::Node & node);

PatrolConfig load_patrol_config(const rclcpp::Node & node);

void validate_patrol_config(const PatrolConfig & config);

}  // namespace auto_patrol
