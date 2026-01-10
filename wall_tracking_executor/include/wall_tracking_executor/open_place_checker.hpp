// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>

#include "wall_tracking_executor/ScanData.hpp"

namespace WallTracking
{
class OpenPlaceChecker{
    public:
    OpenPlaceChecker(
        float detect_angle_start,
        float detect_angle_end,
        int detect_angle_division_num, 
        float range_th, 
        float ratio_th);
    ~OpenPlaceChecker();
    void set_scan(std::shared_ptr<ScanData> & scan);
    bool open_place_detectable(void);
    std::array<float, 3> get_open_laser_info(void);
    // {a_s: angle start, a_e: angle end} of sector
    std::vector<std::array<float, 3>> compute_sector_infos(void);
    float get_sector_angle_nomalize(void);

    protected:
    float detect_angle_start_;
    float detect_angle_end_;
    int detect_angle_division_num_;
    float range_th_;
    float ratio_th_;

    std::vector<std::array<float, 2>> division_angles_;
    std::array<float, 3> open_laser_info_;
    sensor_msgs::msg::LaserScan::SharedPtr filtered_scan_msg_;
    std::shared_ptr<ScanData> scan_;
};
} // namespace lsa_nav_controller