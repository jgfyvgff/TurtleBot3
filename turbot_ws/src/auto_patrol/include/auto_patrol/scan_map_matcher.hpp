#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "auto_patrol/patrol_config.hpp"

namespace auto_patrol
{

// 平面刚体变换，表示 source 坐标系原点及朝向在 target 坐标系中的表达。
struct PlanarTransform
{
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

// 独立于 AMCL 的激光-地图匹配结果。总误差同时包含端点到静态障碍物的距离，
// 以及激光束在到达端点前穿过非自由空间的惩罚；误差越小，几何关系越自洽。
struct ScanMatchResult
{
    PlanarTransform best_pose;
    std::optional<PlanarTransform> runner_up_pose;
    double best_mean_error_m{0.0};
    double best_endpoint_mean_error_m{0.0};
    double best_free_space_mean_penalty_m{0.0};
    double runner_up_mean_error_m{0.0};
    double runner_up_endpoint_mean_error_m{0.0};
    double runner_up_free_space_mean_penalty_m{0.0};
    double runner_up_position_distance_m{0.0};
    double runner_up_yaw_difference_rad{0.0};
    bool map_and_scan_ready{false};
    bool accepted{false};
};

// 使用 OccupancyGrid 距离场评估 LaserScan。该类只保存消息快照和执行确定性计算，
// 不创建 ROS2 Publisher、Timer 或线程，便于在没有 Gazebo/AMCL 的环境中单元测试。
class ScanMapMatcher final
{
public:
    explicit ScanMapMatcher(const PatrolConfig & config);

    void update_map(const nav_msgs::msg::OccupancyGrid & map);
    void update_scan(const sensor_msgs::msg::LaserScan & scan);

    bool has_map() const;
    std::string scan_frame_id() const;
    ScanMatchResult find_global_match(
        const PlanarTransform & base_from_scan) const;
    std::optional<double> score_pose(
        const PlanarTransform & map_from_base,
        const PlanarTransform & base_from_scan) const;

private:
    struct MapSnapshot
    {
        uint32_t width{0U};
        uint32_t height{0U};
        double resolution{0.0};
        PlanarTransform map_from_grid;
        std::vector<int8_t> occupancy;
        std::vector<double> obstacle_distance_m;
        bool valid{false};
    };

    struct Beam
    {
        // sensor_* 与 endpoint_* 都在 base_frame 中。保留真实传感器原点，
        // 才能在 base_frame 与 LaserScan frame 有平移外参时正确检查射线内部空间。
        double sensor_x{0.0};
        double sensor_y{0.0};
        double endpoint_x{0.0};
        double endpoint_y{0.0};
    };

    struct Candidate
    {
        PlanarTransform pose;
        double mean_error_m{0.0};
        double endpoint_mean_error_m{0.0};
        double free_space_mean_penalty_m{0.0};
    };

    struct CandidateScore
    {
        double mean_error_m{0.0};
        double endpoint_mean_error_m{0.0};
        double free_space_mean_penalty_m{0.0};
    };

    std::optional<MapSnapshot> map_snapshot() const;
    std::optional<sensor_msgs::msg::LaserScan> scan_snapshot() const;
    std::vector<Beam> select_beams(
        const sensor_msgs::msg::LaserScan & scan,
        const PlanarTransform & base_from_scan) const;
    std::vector<Candidate> coarse_search(
        const MapSnapshot & map,
        const std::vector<Beam> & beams) const;
    Candidate refine_candidate(
        const MapSnapshot & map,
        const std::vector<Beam> & beams,
        const Candidate & coarse_candidate) const;
    CandidateScore score_candidate(
        const MapSnapshot & map,
        const std::vector<Beam> & beams,
        const PlanarTransform & map_from_base) const;
    bool beam_crosses_nonfree_space(
        const MapSnapshot & map,
        double sensor_x,
        double sensor_y,
        double endpoint_x,
        double endpoint_y) const;
    bool candidates_are_separated(
        const PlanarTransform & first,
        const PlanarTransform & second) const;
    bool world_to_index(
        const MapSnapshot & map,
        double world_x,
        double world_y,
        std::size_t & index) const;

    const PatrolConfig config_;
    mutable std::mutex mutex_;
    std::optional<MapSnapshot> map_;
    std::optional<sensor_msgs::msg::LaserScan> scan_;
};

}  // namespace auto_patrol
