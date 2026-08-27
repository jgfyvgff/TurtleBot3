#include "auto_patrol/scan_map_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <utility>

#include "auto_patrol/pose_utils.hpp"

namespace auto_patrol
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kOutsideMapPenaltyM = 1.0;
constexpr int8_t kOccupiedValue = 65;
// 激光端点本身应落在障碍物表面，因此不把末端约 1.5 个栅格作为“穿墙”证据。
constexpr double kEndpointInteriorClearanceCells = 1.5;
// 只精化粗搜索中误差最低的一小部分候选。这样既避免把整张地图的每个候选都做细粒度
// 搜索，也给粗搜索结果留出足够的候选来纠正网格采样误差。
constexpr std::size_t kRefineCandidateLimit = 12U;

struct DistanceQueueEntry
{
    double distance_m;
    std::size_t index;

    bool operator>(const DistanceQueueEntry & other) const
    {
        return distance_m > other.distance_m;
    }
};

double apply_yaw_x(double yaw, double x, double y)
{
    return std::cos(yaw) * x - std::sin(yaw) * y;
}

double apply_yaw_y(double yaw, double x, double y)
{
    return std::sin(yaw) * x + std::cos(yaw) * y;
}

}  // namespace

ScanMapMatcher::ScanMapMatcher(const PatrolConfig & config)
    : config_(config)
{
}

void ScanMapMatcher::update_map(const nav_msgs::msg::OccupancyGrid & message)
{
    MapSnapshot snapshot;
    snapshot.width = message.info.width;
    snapshot.height = message.info.height;
    snapshot.resolution = static_cast<double>(message.info.resolution);
    snapshot.map_from_grid.x = message.info.origin.position.x;
    snapshot.map_from_grid.y = message.info.origin.position.y;
    snapshot.map_from_grid.yaw = pose_utils::quaternion_to_yaw(
        message.info.origin.orientation);

    const std::size_t cell_count =
        static_cast<std::size_t>(snapshot.width) *
        static_cast<std::size_t>(snapshot.height);
    if (snapshot.width == 0U || snapshot.height == 0U ||
        !std::isfinite(snapshot.resolution) || snapshot.resolution <= 0.0 ||
        message.data.size() != cell_count)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        map_.reset();
        return;
    }

    snapshot.occupancy = message.data;
    snapshot.obstacle_distance_m.assign(
        cell_count,
        std::numeric_limits<double>::infinity());
    std::priority_queue<
        DistanceQueueEntry,
        std::vector<DistanceQueueEntry>,
        std::greater<DistanceQueueEntry>> queue;

    for (std::size_t index = 0U; index < cell_count; ++index) {
        if (snapshot.occupancy[index] >= kOccupiedValue) {
            snapshot.obstacle_distance_m[index] = 0.0;
            queue.push(DistanceQueueEntry{0.0, index});
        }
    }

    if (queue.empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        map_.reset();
        return;
    }

    constexpr int kNeighborCount = 8;
    constexpr int kOffsetX[kNeighborCount] = {-1, 0, 1, -1, 1, -1, 0, 1};
    constexpr int kOffsetY[kNeighborCount] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const double diagonal_step_m = snapshot.resolution * std::sqrt(2.0);

    while (!queue.empty()) {
        const DistanceQueueEntry current = queue.top();
        queue.pop();
        if (current.distance_m != snapshot.obstacle_distance_m[current.index]) {
            continue;
        }

        const int current_x = static_cast<int>(
            current.index % static_cast<std::size_t>(snapshot.width));
        const int current_y = static_cast<int>(
            current.index / static_cast<std::size_t>(snapshot.width));
        for (int neighbor = 0; neighbor < kNeighborCount; ++neighbor) {
            const int neighbor_x = current_x + kOffsetX[neighbor];
            const int neighbor_y = current_y + kOffsetY[neighbor];
            if (neighbor_x < 0 || neighbor_y < 0 ||
                neighbor_x >= static_cast<int>(snapshot.width) ||
                neighbor_y >= static_cast<int>(snapshot.height))
            {
                continue;
            }

            const bool is_diagonal =
                kOffsetX[neighbor] != 0 && kOffsetY[neighbor] != 0;
            const double candidate_distance = current.distance_m +
                (is_diagonal ? diagonal_step_m : snapshot.resolution);
            const std::size_t neighbor_index =
                static_cast<std::size_t>(neighbor_y) *
                static_cast<std::size_t>(snapshot.width) +
                static_cast<std::size_t>(neighbor_x);
            if (candidate_distance < snapshot.obstacle_distance_m[neighbor_index]) {
                snapshot.obstacle_distance_m[neighbor_index] = candidate_distance;
                queue.push(DistanceQueueEntry{
                    candidate_distance,
                    neighbor_index});
            }
        }
    }

    snapshot.valid = true;
    std::lock_guard<std::mutex> lock(mutex_);
    map_ = std::move(snapshot);
}

void ScanMapMatcher::update_scan(const sensor_msgs::msg::LaserScan & scan)
{
    std::lock_guard<std::mutex> lock(mutex_);
    scan_ = scan;
}

bool ScanMapMatcher::has_map() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return map_.has_value() && map_->valid;
}

std::string ScanMapMatcher::scan_frame_id() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return scan_.has_value() ? scan_->header.frame_id : std::string{};
}

ScanMatchResult ScanMapMatcher::find_global_match(
    const PlanarTransform & base_from_scan) const
{
    ScanMatchResult result;
    const auto map = map_snapshot();
    const auto scan = scan_snapshot();
    if (!map.has_value() || !scan.has_value() || !map->valid) {
        return result;
    }

    const std::vector<Beam> beams = select_beams(*scan, base_from_scan);
    if (beams.size() < 3U) {
        return result;
    }

    result.map_and_scan_ready = true;
    std::vector<Candidate> candidates = coarse_search(*map, beams);
    if (candidates.empty()) {
        return result;
    }

    std::sort(
        candidates.begin(),
        candidates.end(),
        [](const Candidate & first, const Candidate & second) {
            return first.mean_error_m < second.mean_error_m;
        });
    std::vector<Candidate> refined_candidates;
    const std::size_t candidate_count = std::min(
        candidates.size(),
        kRefineCandidateLimit);
    refined_candidates.reserve(candidate_count);
    for (std::size_t index = 0U; index < candidate_count; ++index) {
        refined_candidates.push_back(
            refine_candidate(*map, beams, candidates[index]));
    }
    std::sort(
        refined_candidates.begin(),
        refined_candidates.end(),
        [](const Candidate & first, const Candidate & second) {
            return first.mean_error_m < second.mean_error_m;
        });

    const Candidate best = refined_candidates.front();
    result.best_pose = best.pose;
    result.best_mean_error_m = best.mean_error_m;
    result.best_endpoint_mean_error_m = best.endpoint_mean_error_m;
    result.best_free_space_mean_penalty_m = best.free_space_mean_penalty_m;

    // 次优候选必须从完整粗搜索结果中选择，不能因为前若干精化候选都落在同一位置附近，
    // 就把远处的相似区域误认为“不存在”。随后只精化这一个最有竞争力的分离候选。
    const auto runner_up_coarse = std::find_if(
        candidates.begin(),
        candidates.end(),
        [this, &best](const Candidate & candidate) {
            return candidates_are_separated(candidate.pose, best.pose);
        });
    if (runner_up_coarse != candidates.end()) {
        const Candidate runner_up = refine_candidate(
            *map,
            beams,
            *runner_up_coarse);
        result.runner_up_pose = runner_up.pose;
        result.runner_up_mean_error_m = runner_up.mean_error_m;
        result.runner_up_endpoint_mean_error_m =
            runner_up.endpoint_mean_error_m;
        result.runner_up_free_space_mean_penalty_m =
            runner_up.free_space_mean_penalty_m;
        result.runner_up_position_distance_m = std::hypot(
            runner_up.pose.x - best.pose.x,
            runner_up.pose.y - best.pose.y);
        result.runner_up_yaw_difference_rad = std::abs(
            pose_utils::angle_difference(runner_up.pose.yaw, best.pose.yaw));
    } else {
        result.runner_up_mean_error_m = std::numeric_limits<double>::infinity();
    }

    const bool has_clear_margin =
        !std::isfinite(result.runner_up_mean_error_m) ||
        result.runner_up_mean_error_m - result.best_mean_error_m >=
            config_.localization_scan_match_min_margin_m;
    result.accepted =
        result.best_mean_error_m <=
            config_.localization_scan_match_max_mean_error_m &&
        has_clear_margin;
    return result;
}

std::optional<double> ScanMapMatcher::score_pose(
    const PlanarTransform & map_from_base,
    const PlanarTransform & base_from_scan) const
{
    const auto map = map_snapshot();
    const auto scan = scan_snapshot();
    if (!map.has_value() || !scan.has_value() || !map->valid) {
        return std::nullopt;
    }
    const std::vector<Beam> beams = select_beams(*scan, base_from_scan);
    if (beams.size() < 3U) {
        return std::nullopt;
    }
    return score_candidate(*map, beams, map_from_base).mean_error_m;
}

std::optional<ScanMapMatcher::MapSnapshot> ScanMapMatcher::map_snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return map_;
}

std::optional<sensor_msgs::msg::LaserScan> ScanMapMatcher::scan_snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return scan_;
}

std::vector<ScanMapMatcher::Beam> ScanMapMatcher::select_beams(
    const sensor_msgs::msg::LaserScan & scan,
    const PlanarTransform & base_from_scan) const
{
    std::vector<Beam> all_beams;
    const double range_limit = std::min(
        static_cast<double>(scan.range_max),
        config_.localization_scan_match_max_range_m);
    if (!std::isfinite(range_limit) || range_limit <= 0.0 ||
        !std::isfinite(static_cast<double>(scan.angle_increment)))
    {
        return all_beams;
    }

    for (std::size_t index = 0U; index < scan.ranges.size(); ++index) {
        const double range = static_cast<double>(scan.ranges[index]);
        if (!std::isfinite(range) ||
            range <= static_cast<double>(scan.range_min) ||
            range >= range_limit)
        {
            continue;
        }
        const double angle = static_cast<double>(scan.angle_min) +
            static_cast<double>(index) * static_cast<double>(scan.angle_increment);
        const double scan_x = range * std::cos(angle);
        const double scan_y = range * std::sin(angle);
        all_beams.push_back(Beam{
            base_from_scan.x,
            base_from_scan.y,
            base_from_scan.x + apply_yaw_x(base_from_scan.yaw, scan_x, scan_y),
            base_from_scan.y + apply_yaw_y(base_from_scan.yaw, scan_x, scan_y)});
    }

    if (all_beams.size() <=
        static_cast<std::size_t>(config_.localization_scan_match_max_beams))
    {
        return all_beams;
    }

    std::vector<Beam> selected;
    selected.reserve(
        static_cast<std::size_t>(config_.localization_scan_match_max_beams));
    const double interval = static_cast<double>(all_beams.size() - 1U) /
        static_cast<double>(config_.localization_scan_match_max_beams - 1);
    for (int index = 0; index < config_.localization_scan_match_max_beams; ++index) {
        const std::size_t source_index = static_cast<std::size_t>(std::llround(
            static_cast<double>(index) * interval));
        selected.push_back(all_beams[source_index]);
    }
    return selected;
}

std::vector<ScanMapMatcher::Candidate> ScanMapMatcher::coarse_search(
    const MapSnapshot & map,
    const std::vector<Beam> & beams) const
{
    const int cell_step = std::max(
        1,
        static_cast<int>(std::llround(
            config_.localization_scan_match_coarse_step_m / map.resolution)));
    const int yaw_samples = std::max(
        1,
        static_cast<int>(std::ceil(
            (2.0 * kPi) / config_.localization_scan_match_coarse_yaw_step_rad)));
    std::vector<Candidate> candidates;
    candidates.reserve(
        (static_cast<std::size_t>(map.width) / static_cast<std::size_t>(cell_step) + 1U) *
        (static_cast<std::size_t>(map.height) / static_cast<std::size_t>(cell_step) + 1U));

    for (uint32_t row = 0U; row < map.height; row += static_cast<uint32_t>(cell_step)) {
        for (uint32_t column = 0U; column < map.width;
             column += static_cast<uint32_t>(cell_step))
        {
            const std::size_t index =
                static_cast<std::size_t>(row) * static_cast<std::size_t>(map.width) +
                static_cast<std::size_t>(column);
            if (map.occupancy[index] < 0 || map.occupancy[index] >= kOccupiedValue) {
                continue;
            }

            const double grid_x =
                (static_cast<double>(column) + 0.5) * map.resolution;
            const double grid_y =
                (static_cast<double>(row) + 0.5) * map.resolution;
            const double map_x = map.map_from_grid.x +
                apply_yaw_x(map.map_from_grid.yaw, grid_x, grid_y);
            const double map_y = map.map_from_grid.y +
                apply_yaw_y(map.map_from_grid.yaw, grid_x, grid_y);
            for (int yaw_index = 0; yaw_index < yaw_samples; ++yaw_index) {
                const double yaw = -kPi +
                    (2.0 * kPi * static_cast<double>(yaw_index) /
                        static_cast<double>(yaw_samples));
                const PlanarTransform pose{map_x, map_y, yaw};
                const CandidateScore score = score_candidate(map, beams, pose);
                candidates.push_back(Candidate{
                    pose,
                    score.mean_error_m,
                    score.endpoint_mean_error_m,
                    score.free_space_mean_penalty_m});
            }
        }
    }
    return candidates;
}

ScanMapMatcher::Candidate ScanMapMatcher::refine_candidate(
    const MapSnapshot & map,
    const std::vector<Beam> & beams,
    const Candidate & coarse_candidate) const
{
    Candidate best = coarse_candidate;
    const int linear_steps = std::max(
        1,
        static_cast<int>(std::ceil(
            config_.localization_scan_match_coarse_step_m /
            config_.localization_scan_match_refine_step_m)));
    const int yaw_steps = std::max(
        1,
        static_cast<int>(std::ceil(
            config_.localization_scan_match_coarse_yaw_step_rad /
            config_.localization_scan_match_refine_yaw_step_rad)));
    for (int x_step = -linear_steps; x_step <= linear_steps; ++x_step) {
        for (int y_step = -linear_steps; y_step <= linear_steps; ++y_step) {
            for (int yaw_step = -yaw_steps; yaw_step <= yaw_steps; ++yaw_step) {
                const PlanarTransform pose{
                    coarse_candidate.pose.x +
                        static_cast<double>(x_step) *
                            config_.localization_scan_match_refine_step_m,
                    coarse_candidate.pose.y +
                        static_cast<double>(y_step) *
                            config_.localization_scan_match_refine_step_m,
                    pose_utils::normalize_angle(
                        coarse_candidate.pose.yaw +
                        static_cast<double>(yaw_step) *
                            config_.localization_scan_match_refine_yaw_step_rad)};
                const CandidateScore score = score_candidate(map, beams, pose);
                if (score.mean_error_m < best.mean_error_m) {
                    best = Candidate{
                        pose,
                        score.mean_error_m,
                        score.endpoint_mean_error_m,
                        score.free_space_mean_penalty_m};
                }
            }
        }
    }
    return best;
}

ScanMapMatcher::CandidateScore ScanMapMatcher::score_candidate(
    const MapSnapshot & map,
    const std::vector<Beam> & beams,
    const PlanarTransform & map_from_base) const
{
    CandidateScore score;
    std::size_t base_index = 0U;
    if (!world_to_index(map, map_from_base.x, map_from_base.y, base_index) ||
        map.occupancy[base_index] < 0 || map.occupancy[base_index] >= kOccupiedValue)
    {
        score.mean_error_m = kOutsideMapPenaltyM;
        score.endpoint_mean_error_m = kOutsideMapPenaltyM;
        return score;
    }

    double endpoint_error_sum_m = 0.0;
    for (const Beam & beam : beams) {
        const double endpoint_x = map_from_base.x +
            apply_yaw_x(
                map_from_base.yaw,
                beam.endpoint_x,
                beam.endpoint_y);
        const double endpoint_y = map_from_base.y +
            apply_yaw_y(
                map_from_base.yaw,
                beam.endpoint_x,
                beam.endpoint_y);
        std::size_t endpoint_index = 0U;
        if (!world_to_index(map, endpoint_x, endpoint_y, endpoint_index) ||
            map.occupancy[endpoint_index] < 0)
        {
            endpoint_error_sum_m += kOutsideMapPenaltyM;
            continue;
        }
        endpoint_error_sum_m += std::min(
            map.obstacle_distance_m[endpoint_index],
            kOutsideMapPenaltyM);
    }

    score.endpoint_mean_error_m = endpoint_error_sum_m /
        static_cast<double>(beams.size());
    // 已超过接受上限的候选无论自由空间是否满足都不可能启动巡检。
    // 先在这里剪枝，避免全图粗搜索为大量明显错误的候选遍历射线路径。
    if (score.endpoint_mean_error_m >
        config_.localization_scan_match_max_mean_error_m)
    {
        score.mean_error_m = score.endpoint_mean_error_m;
        return score;
    }

    const std::size_t free_space_beam_count = std::min(
        beams.size(),
        static_cast<std::size_t>(config_.localization_scan_match_free_space_max_beams));
    std::size_t blocked_free_space_beam_count = 0U;
    // 端点“靠近墙”不足以排除重复走廊或相似房间。另一候选若把激光束中段放到
    // 墙内，便与物理测距矛盾，即使它的端点同样靠近障碍物也不能作为等价解。
    // 只抽样有限束数，保持全图粗搜索的计算上界；抽样位置由输入 beams 的均匀采样保证。
    for (std::size_t selected_index = 0U;
         selected_index < free_space_beam_count;
         ++selected_index)
    {
        const std::size_t beam_index = free_space_beam_count == 1U ? 0U :
            static_cast<std::size_t>(std::llround(
                static_cast<double>(selected_index) *
                static_cast<double>(beams.size() - 1U) /
                static_cast<double>(free_space_beam_count - 1U)));
        const Beam & beam = beams[beam_index];
        const double sensor_x = map_from_base.x +
            apply_yaw_x(
                map_from_base.yaw,
                beam.sensor_x,
                beam.sensor_y);
        const double sensor_y = map_from_base.y +
            apply_yaw_y(
                map_from_base.yaw,
                beam.sensor_x,
                beam.sensor_y);
        const double endpoint_x = map_from_base.x +
            apply_yaw_x(
                map_from_base.yaw,
                beam.endpoint_x,
                beam.endpoint_y);
        const double endpoint_y = map_from_base.y +
            apply_yaw_y(
                map_from_base.yaw,
                beam.endpoint_x,
                beam.endpoint_y);
        if (beam_crosses_nonfree_space(
                map,
                sensor_x,
                sensor_y,
                endpoint_x,
                endpoint_y))
        {
            ++blocked_free_space_beam_count;
        }
    }

    score.free_space_mean_penalty_m =
        config_.localization_scan_match_free_space_penalty_m *
        static_cast<double>(blocked_free_space_beam_count) /
        static_cast<double>(free_space_beam_count);
    score.mean_error_m = score.endpoint_mean_error_m +
        score.free_space_mean_penalty_m;
    return score;
}

bool ScanMapMatcher::beam_crosses_nonfree_space(
    const MapSnapshot & map,
    double sensor_x,
    double sensor_y,
    double endpoint_x,
    double endpoint_y) const
{
    const double delta_x = endpoint_x - sensor_x;
    const double delta_y = endpoint_y - sensor_y;
    const double beam_length_m = std::hypot(delta_x, delta_y);
    const double checked_length_m = beam_length_m -
        kEndpointInteriorClearanceCells * map.resolution;
    if (checked_length_m <= 0.0) {
        return false;
    }

    std::size_t sensor_index = 0U;
    if (!world_to_index(map, sensor_x, sensor_y, sensor_index) ||
        map.occupancy[sensor_index] < 0 ||
        map.occupancy[sensor_index] >= kOccupiedValue)
    {
        return true;
    }

    const int sample_count = std::max(
        1,
        static_cast<int>(std::floor(checked_length_m / map.resolution)));
    for (int sample_index = 1; sample_index <= sample_count; ++sample_index) {
        const double progress_m = std::min(
            checked_length_m,
            static_cast<double>(sample_index) * map.resolution);
        const double ratio = progress_m / beam_length_m;
        std::size_t sample_cell_index = 0U;
        if (!world_to_index(
                map,
                sensor_x + delta_x * ratio,
                sensor_y + delta_y * ratio,
                sample_cell_index) ||
            map.occupancy[sample_cell_index] < 0 ||
            map.occupancy[sample_cell_index] >= kOccupiedValue)
        {
            return true;
        }
    }
    return false;
}

bool ScanMapMatcher::candidates_are_separated(
    const PlanarTransform & first,
    const PlanarTransform & second) const
{
    const double position_distance = std::hypot(
        first.x - second.x,
        first.y - second.y);
    const double yaw_difference = std::abs(pose_utils::angle_difference(
        first.yaw,
        second.yaw));
    return position_distance >= config_.localization_scan_match_min_separation_m ||
        yaw_difference >= config_.localization_scan_match_min_yaw_separation_rad;
}

bool ScanMapMatcher::world_to_index(
    const MapSnapshot & map,
    double world_x,
    double world_y,
    std::size_t & index) const
{
    const double relative_x = world_x - map.map_from_grid.x;
    const double relative_y = world_y - map.map_from_grid.y;
    const double grid_x = apply_yaw_x(
        -map.map_from_grid.yaw,
        relative_x,
        relative_y);
    const double grid_y = apply_yaw_y(
        -map.map_from_grid.yaw,
        relative_x,
        relative_y);
    const int column = static_cast<int>(std::floor(grid_x / map.resolution));
    const int row = static_cast<int>(std::floor(grid_y / map.resolution));
    if (column < 0 || row < 0 ||
        column >= static_cast<int>(map.width) ||
        row >= static_cast<int>(map.height))
    {
        return false;
    }
    index = static_cast<std::size_t>(row) * static_cast<std::size_t>(map.width) +
        static_cast<std::size_t>(column);
    return true;
}

}  // namespace auto_patrol
