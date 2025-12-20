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
    float rotating_search_angle_abs_small, 
    float rotating_search_angle_abs_big, 
    float slow_down_obs_th, 
    float stop_obs_th, 
    float target_wall_dist, 
    float var_th
)
: lin_max_vel_(lin_max_vel), 
  ang_max_vel_(ang_max_vel), 
  kp_(kp), 
  front_search_angle_start_(front_search_angle_start), 
  front_search_angle_end_(front_search_angle_end), 
  side_search_angle_start_(side_search_angle_start), 
  side_search_angle_end_(side_search_angle_end), 
  rotating_search_angle_abs_small_(rotating_search_angle_abs_small), 
  rotating_search_angle_abs_big_(rotating_search_angle_abs_big), 
  slow_down_obs_th_(slow_down_obs_th),
  stop_obs_th_(stop_obs_th), 
  target_wall_dist_(target_wall_dist), 
  var_th_(var_th), 
  range_th_(stop_obs_th+target_wall_dist),
  rotating_(false), 
  rotate_right_(false)
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
    // RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "min range: %f", min_range);
    if(min_range < slow_down_obs_th_){
        // RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "Avoidance");
        cur_cmd[0] = lin_max_vel_ * (fabs(cos(min_ang) * min_range) - 0.3) / slow_down_obs_th_;
        if(!rotating_) {
            rotating_ = true;
            float right_space = scan_->get_side_lat_stat(
                -rotating_search_angle_abs_big_, -rotating_search_angle_abs_small_)[0];
            float left_space = scan_->get_side_lat_stat(
                rotating_search_angle_abs_small_, rotating_search_angle_abs_big_)[0];
            rotate_right_ = right_space >= left_space;
        }
        cur_cmd[1] = rotate_right_ ? -ang_max_vel_ : ang_max_vel_;
    }else{
        rotating_ = false;
        // get side lateral average
        // RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "Wall Following");
        std::array<float, 2> side_stat 
            = scan_->get_side_lat_stat(side_search_angle_start_, side_search_angle_end_);
        float mean = side_stat[0], var = side_stat[1];
        // RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "Mean: %f, Var: %f", mean, var);
        if(var > var_th_){
            std::array<float, 2> min_laser = scan_->get_minimum_laser(side_search_angle_start_, side_search_angle_end_);
            cur_cmd = {lin_max_vel_, get_ang_vel(fabs(sin(min_laser[0]) * min_laser[1]))};
        }else if(mean > range_th_){
            cur_cmd = {lin_max_vel_, 0.0};
        }else{
            cur_cmd = {lin_max_vel_, get_ang_vel(mean)};
        }
    }
    cur_cmd[0] = std::clamp(cur_cmd[0], (float)0., lin_max_vel_);
    cur_cmd[1] = std::clamp(cur_cmd[1], -ang_max_vel_, ang_max_vel_);
    return cur_cmd;
}

float Controller::get_ang_vel(float side_ave)
{
    float error = target_wall_dist_ - side_ave;
    return -kp_ * error;
}

Controller::~Controller(){}
}