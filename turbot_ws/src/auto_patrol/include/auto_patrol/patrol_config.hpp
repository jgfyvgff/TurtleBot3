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
    // 静态地图由 map_server 以 transient-local QoS 发布，默认话题与 Nav2 保持一致。
    std::string localization_map_topic{"/map"};
    // 扫描匹配输出的是该机体坐标系在 map 中的位姿；默认与 AMCL base_frame_id 保持一致。
    std::string localization_base_frame{"base_footprint"};

    // 默认让 AMCL 在地图上全局初始化粒子，不要求预先知道机器人坐标。
    bool automatic_global_localization{true};
    double localization_timeout_sec{120.0};
    double localization_position_variance_threshold{0.25};
    double localization_yaw_variance_threshold{0.10};
    // 两个服务名可配置，便于有命名空间的 Nav2 部署；默认值匹配当前 TurtleBot3 仿真。
    std::string localization_global_localization_service{
        "/reinitialize_global_localization"};
    // 通过 AMCL 的无运动更新服务触发当前静态 LaserScan 的粒子滤波更新，节点不发布速度。
    std::string localization_nomotion_update_service{"/request_nomotion_update"};
    // 两次无运动更新请求之间的最小间隔，单位为 s，过小会无意义地堆积服务请求。
    double localization_nomotion_update_period_sec{0.5};
    // 成功前必须存在 goal_frame -> localization_odom_frame，默认即 map -> odom。
    std::string localization_odom_frame{"odom"};
    // 满足置信度后继续静止确认的时间，单位为 s。
    double localization_settle_duration_sec{1.0};

    // 无运动全局定位先独立匹配静态地图和 LaserScan，再将候选位姿写入 /initialpose。
    // 这防止 AMCL 在相似区域形成低协方差但错误的局部模式后直接启动巡检。
    bool localization_scan_match_enabled{true};
    double localization_scan_match_coarse_step_m{0.20};
    double localization_scan_match_coarse_yaw_step_rad{0.261799};
    double localization_scan_match_refine_step_m{0.05};
    double localization_scan_match_refine_yaw_step_rad{0.034907};
    int localization_scan_match_max_beams{90};
    double localization_scan_match_max_range_m{3.0};
    // 自由空间检查只均匀抽取部分激光束，防止全图搜索对每个候选做全部射线遍历而过慢。
    int localization_scan_match_free_space_max_beams{12};
    // 单束激光在端点之前穿过静态占据或未知栅格时加入的误差，单位 m。
    double localization_scan_match_free_space_penalty_m{0.25};
    double localization_scan_match_max_mean_error_m{0.10};
    double localization_scan_match_min_margin_m{0.05};
    double localization_scan_match_min_separation_m{0.50};
    double localization_scan_match_min_yaw_separation_rad{0.523599};
    double localization_scan_match_pose_tolerance_m{0.20};
    double localization_scan_match_yaw_tolerance_rad{0.20};
    int localization_scan_match_max_initial_pose_retries{1};

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
