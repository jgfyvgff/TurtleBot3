#include "auto_patrol/localization_safety.hpp"

#include <cmath>
#include <cstddef>

namespace auto_patrol::localization_safety
{

bool scan_has_safe_front_clearance(
    const sensor_msgs::msg::LaserScan & scan,
    double minimum_clearance,
    double sector_half_angle)
{
    if (!std::isfinite(minimum_clearance) || minimum_clearance <= 0.0 ||
        !std::isfinite(sector_half_angle) || sector_half_angle <= 0.0 ||
        !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_increment) ||
        !std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) ||
        scan.range_min < 0.0F || scan.range_max <= scan.range_min)
    {
        return false;
    }

    bool has_valid_front_measurement = false;
    for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
        const double angle = static_cast<double>(scan.angle_min) +
            static_cast<double>(index) * static_cast<double>(scan.angle_increment);
        if (!std::isfinite(angle) || std::abs(angle) > sector_half_angle) {
            continue;
        }

        const float range = scan.ranges[index];
        if (!std::isfinite(range) || range < scan.range_min ||
            range > scan.range_max)
        {
            continue;
        }

        has_valid_front_measurement = true;
        if (static_cast<double>(range) <= minimum_clearance) {
            return false;
        }
    }

    return has_valid_front_measurement;
}

double planar_distance(
    double start_x,
    double start_y,
    double current_x,
    double current_y)
{
    return std::hypot(current_x - start_x, current_y - start_y);
}

}  // namespace auto_patrol::localization_safety
