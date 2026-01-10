// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>

#include "wall_tracking_executor/open_place_checker.hpp"
// #include "lsa_nav_controller/utils/scan_->tils.hpp"

namespace WallTracking
{
OpenPlaceChecker::OpenPlaceChecker(
    float detect_angle_start,
    float detect_angle_end,
    int detect_angle_division_num, 
    float range_th, 
    float ratio_th)
: detect_angle_start_(detect_angle_start), 
  detect_angle_end_(detect_angle_end),
  detect_angle_division_num_(detect_angle_division_num), 
  range_th_(range_th),
  ratio_th_(ratio_th)
{
    division_angles_.reserve(detect_angle_division_num_);
    float angle_range = detect_angle_end_ - detect_angle_start_;
    float division_size = angle_range / static_cast<float>(detect_angle_division_num_);
    for(int i=0; i<detect_angle_division_num_; ++i){
        float start_angle = detect_angle_start_ + i * division_size;
        float end_angle = start_angle + division_size;
        division_angles_.emplace_back(std::array<float, 2>{start_angle, end_angle});
    }
}

void OpenPlaceChecker::set_scan(std::shared_ptr<ScanData> & scan)
{
    scan_ = scan;
    // RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), 
    //     "Scan data set: frame_id: %s, angle_min: %.2f [deg], angle_max: %.2f [deg], angle_increment: %.2f [deg], range_min: %.2f [m], range_max: %.2f [m], num_ranges: %zu", 
    //     scan_->frame_id_.c_str(), scan_->angle_min_*180.0/M_PI, scan_->angle_max_*180.0/M_PI, scan_->angle_increment_*180.0/M_PI, scan_->range_min_, scan_->range_max_, scan_->ranges_.size());
}

bool OpenPlaceChecker::open_place_detectable(void)
{
    // Scan info
    // float angle_increment = scan_->angle_increment_;
    // auto angle_min = scan_->angle_min_;

    // Define for storing scores
    std::vector<std::array<float, 3>> sector_infos = compute_sector_infos();
    open_laser_info_ = sector_infos[0];

    // Define for storing scores
    // float max_ratio_score = -1.0f;
    // std::array<float, 3> open_laser_info = sector_infos[0];

    // open_laser_info_ = {division_angles_[0][0], 0., -1.0};

    // Evaluate each sector
    for(size_t i=1; i < sector_infos.size(); ++i){
    // for(auto angles : division_angles_){
        // Get sector info
        // RCLCPP_INFO(rclcpp::get_logger("wall_tracking_executor"), 
        //     "start: %f, end: %f", angles[0], angles[1]);
        // [0]: direction, [1]: distance, [2]: ratio score
        std::array<float, 3> tmp_sector_info = sector_infos[i];
        // RCLCPP_INFO(rclcpp::get_logger("wall_tracking_executor"), 
        //     "direction: %f, distance: %f, ratio: %f", 
        //     tmp_sector_info[0], tmp_sector_info[1], tmp_sector_info[2]);
        // Compare temporary scores and max scores
        // tmp_sector_ratio is higher than max_ratio_score
        bool higher_than_max = tmp_sector_info[2] > open_laser_info_[2];

        // tmp_sector_ratio is same as max_ratio_score
        // tmp_representative_angle absolute value is closer to zero than max_representative_angle
        bool same_and_near_zero_than_max = (tmp_sector_info[2] == open_laser_info_[2]) && 
            (std::abs(tmp_sector_info[0]) < std::abs(open_laser_info_[0]));
        
        if(higher_than_max || same_and_near_zero_than_max){
            // open_laser_info_[2] = tmp_sector_info[2];
            std::copy(tmp_sector_info.begin(), tmp_sector_info.end(), open_laser_info_.begin());
        }
    }
    RCLCPP_DEBUG(rclcpp::get_logger("wall_tracking_node"), 
        "Max Score = {direction: %f, distance: %f, ratio: %f}", 
        open_laser_info_[0], open_laser_info_[1], open_laser_info_[2]);
    // if max score is less than ratio_th, return false
    if(open_laser_info_[2] < ratio_th_) return false; // No open place
    else return true;
}

// {a_s: angle start, a_e: angle end} of sector
std::vector<std::array<float, 3>> OpenPlaceChecker::compute_sector_infos(void)
{
    std::vector<std::array<float, 3>> sector_infos;
    sector_infos.reserve(detect_angle_division_num_);

    // Evaluate each sector
    for(auto angles : division_angles_){
        // Compute representative angle of the sector
        float a_s = angles[0], a_e = angles[1];
        float representative_angle = (a_s + a_e) / 2.0f;

        // Get indices
        size_t i_s= scan_->rad2index(a_s); 
        size_t i_e= scan_->rad2index(a_e);
            
        // Compute average range and ratio score
        float sum_l=0.0f, sum_n=0.0f, sum_i=0.0f;
        for(size_t i=i_s; i<=i_e; ++i){
            float range = scan_->ranges_[i];
            if(!std::isnan(range)){
                if(range >= range_th_){
                    sum_l += range;
                    ++sum_n;
                }
                ++sum_i;
            }
        }
        // RCLCPP_INFO(rclcpp::get_logger("wall_tracking_executor"), 
        //     "direction: %f, sum_l: %f, sum_n: %f, sum_i: %f", 
        //     representative_angle, sum_l, sum_n, sum_i);

        // [0]: direction, [1]: distance, [2]: ratio score
        std::array<float, 3> sector_info = {
            representative_angle, 
            sum_n > 0.f ? static_cast<float>(sum_l / sum_n) : 0.f, 
            sum_i > 0.f ? static_cast<float>(sum_n / sum_i) : 0.f
        };
        sector_infos.emplace_back(sector_info);
    }
    return sector_infos;
}

// [0]: direction, [1]: distance, [2]: ratio score
std::array<float, 3> OpenPlaceChecker::get_open_laser_info(void){return open_laser_info_;}

float OpenPlaceChecker::get_sector_angle_nomalize(void){
    return open_laser_info_[0] / fabs(detect_angle_start_);
}

OpenPlaceChecker::~OpenPlaceChecker(){}

} // namespace lsa_nav_controller