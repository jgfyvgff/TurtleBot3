#pragma once

#include "sensor_msgs/msg/laser_scan.hpp"

namespace auto_patrol::localization_safety
{

// 只处理一帧 LaserScan 的纯计算。前方扇区不存在有效量测时返回 false，
// 因为在未知净空下前进无法构成安全的定位验证。
bool scan_has_safe_front_clearance(
    const sensor_msgs::msg::LaserScan & scan,
    double minimum_clearance,
    double sector_half_angle);

// 使用二维 odom 平面距离作为短距离验证的真实停止条件。
double planar_distance(
    double start_x,
    double start_y,
    double current_x,
    double current_y);

}  // namespace auto_patrol::localization_safety
