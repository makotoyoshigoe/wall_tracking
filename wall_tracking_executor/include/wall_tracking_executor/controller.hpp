// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#pragma once 

#include "wall_tracking_executor/ScanData.hpp"

namespace WallTracking {
class Controller{
    public: 
    Controller(
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
    );
    void set_scan(
        std::shared_ptr<ScanData> & scan);
    std::array<float, 2> get_cmd_vel(void);
    std::array<float, 2> wall_tracking(void);
    std::array<float, 2> open_place_navigation(void);
    ~Controller();
    float get_ang_vel(float side_ave);

    private:
        float lin_max_vel_; 
        float ang_max_vel_;
        float kp_;
        float front_search_angle_start_;
        float front_search_angle_end_;
        float side_search_angle_start_; 
        float side_search_angle_end_;
        float rotating_search_angle_abs_small_;
        float rotating_search_angle_abs_big_;
        float slow_down_obs_th_;
        float stop_obs_th_;
        float target_wall_dist_;
        float var_th_;
        float range_th_;

        std::shared_ptr<ScanData> scan_;
        bool outdoor_;
        bool rotating_;
        bool rotate_right_;
};

}