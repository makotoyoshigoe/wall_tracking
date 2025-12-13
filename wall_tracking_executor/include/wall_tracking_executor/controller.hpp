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
        float obs_avoid_th, 
        float target_wall_dist
    );
    void set_scan(
        std::shared_ptr<ScanData> & scan);
    std::array<float, 2> get_cmd_vel(void);
    std::array<float, 2> wall_tracking(void);
    std::array<float, 2> open_place_navigation(void);
    ~Controller();
    float get_lin_vel(float ave_front);
    float get_ang_vel(float side_ave);

    private:
        float lin_max_vel_; 
        float ang_max_vel_;
        float kp_;
        float front_search_angle_start_;
        float front_search_angle_end_;
        float side_search_angle_start_; 
        float side_search_angle_end_;
        float obs_avoid_th_;
        float target_wall_dist_;
        std::shared_ptr<ScanData> scan_;
        bool outdoor_;
};

}