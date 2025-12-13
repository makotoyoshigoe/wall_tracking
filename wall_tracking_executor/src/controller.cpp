// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>

#include "wall_tracking_executor/controller.hpp"

namespace WallTracking {

Controller::Controller(
    float lin_max_vel, 
    float ang_max_vel, 
    float kp, 
    float front_search_angle_start, 
    float front_search_angle_end, 
    float side_search_angle_start, 
    float side_search_angle_end, 
    float obs_avoid_th, 
    float target_wall_dist
)
: lin_max_vel_(lin_max_vel), 
  ang_max_vel_(ang_max_vel), 
  kp_(kp), 
  front_search_angle_start_(front_search_angle_start), 
  front_search_angle_end_(front_search_angle_end), 
  side_search_angle_start_(side_search_angle_start), 
  side_search_angle_end_(side_search_angle_end), 
  obs_avoid_th_(obs_avoid_th), 
  target_wall_dist_(target_wall_dist)
{

}

void Controller::set_scan(std::shared_ptr<ScanData> & scan){scan_ = scan;}

std::array<float, 2> Controller::get_cmd_vel(void)
{
    // 前方の障害物との最小距離を取得
    // [0]: angle, [1]: range
    std::array<float, 2> min_front_laser = scan_->get_minimum_laser(front_search_angle_start_, front_search_angle_end_);

    std::array<float, 2> cur_cmd;
    float min_ang = min_front_laser[0];
    float min_range = min_front_laser[1];
    RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "min range: %f", min_range);
    if(min_range < obs_avoid_th_){
        RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "Avoidance");
        cur_cmd[0] = lin_max_vel_ * (min_range - 0.3) / obs_avoid_th_;
        cur_cmd[1] = -ang_max_vel_;
    }else{
        // get side lateral average
        RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "Wall Following");
        float side_ave 
            = scan_->get_side_lat_ave(side_search_angle_start_, side_search_angle_end_);
        cur_cmd = {lin_max_vel_, get_ang_vel(side_ave)};
    }
    cur_cmd[0] = std::clamp(cur_cmd[0], (float)0., lin_max_vel_);
    cur_cmd[1] = std::clamp(cur_cmd[1], -ang_max_vel_, ang_max_vel_);
    return cur_cmd;
}
float Controller::get_lin_vel(float ave_front)
{
    return lin_max_vel_;
}

float Controller::get_ang_vel(float side_ave)
{
    float error = target_wall_dist_ - side_ave;
    return -kp_ * error;
}

Controller::~Controller(){}
}