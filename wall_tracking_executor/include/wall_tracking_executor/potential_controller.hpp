// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "wall_tracking_executor/ScanData.hpp"
// #include "lsa_nav_controller/map/map.hpp"

namespace WallTracking
{

struct Force{
    float x = 0.0f;
    float y = 0.0f;
    Force operator+(const Force & other) const
    {
        return {x + other.x, y + other.y};
    }
    Force operator-(const Force & other) const
    {
        return {x - other.x, y - other.y};
    }
};

class PotentialController{
    public:
    // constructor/destructor
    PotentialController(
        float critical_distance, 
        float repulsive_gain, 
        float attractive_gain, 
        float min_distance, 
        float max_linear_vel,
        float max_angular_vel, 
        float sigma);
    ~PotentialController();

    // setters
    void set_scan(
        //std::shared_ptr<Scan> & scan, 
        std::shared_ptr<ScanData> & scan); 
    
    // main methods
    std::array<float, 2> get_cmd_vel(
        std::array<float, 3> open_laser_info);
    std::array<float, 2> force_to_cmd_vel(Force force);
    Force calc_potential_force(void);
    Force calc_repulsive_force(void);
    Force calc_attractive_force(void);
    void nomalize(float x_in, float y_in, float & x_out, float & y_out);

    private:
    // std::vector<std::array<float, 4>> neighborhood_obs_;
    std::shared_ptr<ScanData> scan_;

    // parameters
    float critical_distance_;
    float sigma_;
    float repulsive_gain_;
    float attractive_gain_;
    float detect_angle_start_;
    float detect_angle_end_;
    float min_distance_; // for preventing division near zero
    float max_linear_vel_;
    float max_angular_vel_;
    int detect_angle_division_num_;
    float robot_radius_;

    // [0]: direction, [1]: distance, [2]: ratio score
    std::array<float, 3> open_laser_info_;
};

} // namespace lsa_nav_controller