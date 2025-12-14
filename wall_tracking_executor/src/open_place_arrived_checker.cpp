// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>

#include "wall_tracking_executor/open_place_arrived_checker.hpp"

namespace WallTracking
{
OpenPlaceArrivedChecker::OpenPlaceArrivedChecker(
    float detect_angle_start,
    float detect_angle_end,
    int detect_angle_division_num, 
    float range_th, 
    float ratio_th)
: OpenPlaceChecker(
  detect_angle_start, 
  detect_angle_end,
  detect_angle_division_num, 
  range_th,
  ratio_th
)
{
}

// [0]: direction, [1]: distance, [2]: ratio score
bool OpenPlaceArrivedChecker::open_place_arrived(void)
{
    std::vector<std::array<float, 3>> sector_infos = compute_sector_infos();
    for(auto &sector_info : sector_infos){
        if(sector_info[2] < ratio_th_) return false;
    }
    return true;
}

OpenPlaceArrivedChecker::~OpenPlaceArrivedChecker(){}

} // namespace lsa_nav_controller