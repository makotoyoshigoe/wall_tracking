// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>

#include "wall_tracking_executor/potential_controller.hpp"

namespace WallTracking
{
PotentialController::PotentialController(
    float critical_distance, 
    float repulsive_gain, 
    float attractive_gain, 
    float min_distance,
    float max_linear_vel,
    float max_angular_vel, 
    float sigma):
critical_distance_(critical_distance), 
repulsive_gain_(repulsive_gain),
attractive_gain_(attractive_gain),
min_distance_(min_distance),
max_linear_vel_(max_linear_vel),
max_angular_vel_(max_angular_vel),
sigma_(sigma)
{
}

void PotentialController::set_scan(
    std::shared_ptr<ScanData> & scan) 
{
    scan_ = scan;
    //RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "Set Scan Data");
}

std::array<float, 2> PotentialController::get_cmd_vel(
    std::array<float, 3> open_laser_info, 
    float lin_vel_coeff)
{
    std::copy(open_laser_info.begin(), open_laser_info.end(), std::begin(open_laser_info_));
    std::array<float, 2> cmd_vel = force_to_cmd_vel(calc_potential_force());
    cmd_vel[0] *= lin_vel_coeff;
    return cmd_vel;
}

std::array<float, 2> PotentialController::get_cmd_vel(
    float nomalize_angle)
{
    return {max_linear_vel_, nomalize_angle * fabs(max_angular_vel_)};
}

std::array<float, 2> PotentialController::force_to_cmd_vel(Force force)
{
    float force_magnitude = hypotf(force.x, force.y); 
    float theta_force = atan2f(force.y, force.x);
    // RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), 
    //             "Force mag: %.3f, theta: %.3f, Force X: %.3f, Force Y: %.3f", 
    //             force_magnitude, theta_force, force.x, force.y);

    float tmp_linear_vel = force_magnitude > 1e-3 ? force.x / force_magnitude * max_linear_vel_ : 0.0f;
    float tmp_ang_vel_vel = fabs(theta_force) > 1e-3 ? theta_force / fabs(theta_force) : 0.0f;
    return {
        std::clamp(tmp_linear_vel, (float)0., max_linear_vel_), 
        std::clamp(tmp_ang_vel_vel, -max_angular_vel_, max_angular_vel_)};
}

Force PotentialController::calc_potential_force(void)
{
    Force repulsive_force = calc_repulsive_force();
    Force attractive_force = calc_attractive_force();
    return attractive_force + repulsive_force;
}

Force PotentialController::calc_repulsive_force(void)
{
    float rx = 0.0f, ry = 0.0f;
    float angle = scan_->angle_min_;
    for(auto & range : scan_->ranges_){
        if(range < critical_distance_){
            // Force vector
            float ex = cos(angle), ey = sin(angle);

            // Force magnitude
            float mag = repulsive_gain_ * exp(-range / sigma_);

            // Accumulate force
            rx += mag * (-ex);
            ry += mag * (-ey);

        }
        angle += scan_->angle_increment_;
    }
    return {rx, ry};
}

Force PotentialController::calc_attractive_force(void)
{
    float t = open_laser_info_[0], r = open_laser_info_[1];
    float fx = r * cosf(t), fy = r * sinf(t);
    return {fx * attractive_gain_, fy * attractive_gain_};
}

PotentialController::~PotentialController(){}

} // namespace lsa_nav_controller