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

bool OpenPlaceChecker::is_open_place_available(void)
{
    // Scan info
    float angle_increment = scan_->angle_increment_;
    auto angle_min = scan_->angle_min_;

    // Define for storing scores
    float max_ratio_score = -1.0f;

    open_laser_info_ = {division_angles_[0][0], 0., -1.0};

    // Evaluate each sector
    for(auto angles : division_angles_){
        // Get sector info
        // RCLCPP_INFO(rclcpp::get_logger("wall_tracking_executor"), 
        //     "start: %f, end: %f", angles[0], angles[1]);
        // [0]: direction, [1]: distance, [2]: ratio score
        std::array<float, 3> tmp_sector_info = compute_sector_info(angles[0], angles[1]);
        RCLCPP_INFO(rclcpp::get_logger("wall_tracking_executor"), 
            "direction: %f, distance: %f, ratio: %f", 
            tmp_sector_info[0], tmp_sector_info[1], tmp_sector_info[2]);
        // Compare temporary scores and max scores
        // tmp_sector_ratio is higher than max_ratio_score
        bool higher_than_max = tmp_sector_info[2] > max_ratio_score;

        // tmp_sector_ratio is same as max_ratio_score
        // tmp_representative_angle absolute value is closer to zero than max_representative_angle
        bool same_and_near_zero_than_max = (tmp_sector_info[2] == max_ratio_score) && 
            (std::abs(tmp_sector_info[0]) < std::abs(open_laser_info_[0]));
        
        if(higher_than_max || same_and_near_zero_than_max){
            max_ratio_score = tmp_sector_info[2];
            std::copy(tmp_sector_info.begin(), tmp_sector_info.end(), open_laser_info_.begin());
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("wall_tracking_executor"), 
        "Max Score = {direction: %f, distance: %f, ratio: %f}", 
        open_laser_info_[0], open_laser_info_[1], open_laser_info_[2]);
    // if max score is less than ratio_th, return false
    if(max_ratio_score < ratio_th_) return false; // No open
    else return true;
}

std::array<float, 3> OpenPlaceChecker::compute_sector_info(
    float a_s, float a_e) // {a_s: angle start, a_e: angle end} of sector
{
    // Compute representative angle of the sector
    float representative_angle = (a_s + a_e) / 2.0f;

    // Get indices
    float start = a_s, end = a_e;
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
    return {
        representative_angle,
        sum_n > 0 ? static_cast<float>(sum_l / sum_n) : 0., 
        sum_i > 0 ? static_cast<float>(sum_n / sum_i) : 0.,
    };
}

// [0]: direction, [1]: distance, [2]: ratio score
std::array<float, 3> OpenPlaceChecker::get_open_laser_info(void){return open_laser_info_;}

OpenPlaceChecker::~OpenPlaceChecker(){}

} // namespace lsa_nav_controller