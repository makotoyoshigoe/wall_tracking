// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#ifndef SCANDATA__SCANDATA_HPP_
#define SCANDATA__SCANDATA_HPP_

#define DEG2RAD(deg) ((deg)*M_PI/180)
#define RAD2DEG(rad) ((rad)*180/M_PI)

#include <vector>
#include <cmath>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace WallTracking{
class ScanData
{
private:
    std::string frame_id_;
    sensor_msgs::msg::LaserScan::ConstSharedPtr tmp_scan_msg_;

    // Open Place Checker
    float detect_angle_start_;
    float detect_angle_end_;
    int detect_angle_division_num_;
    float range_th_;
    float ratio_th_;

public:
    ScanData(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
    ScanData(void);
    ~ScanData();
    void dataUpdate(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
    float frontWallCheck(float start_deg, float threshold);
    float leftWallCheck(float start_deg, float end_deg);
    void openPlaceCheck(float start_deg, float end_deg, float threshold, float &per, float &mean_l);
    bool conflictCheck(float deg, float threshold);
    bool thresholdCheck(float deg, float threshold);
    bool noiseCheck(float deg);
    int deg2index(float deg);
    float index2deg(int index);
    float index2rad(int index);

    // New methods
    std::array<float, 2> get_minimum_laser(float start_angle, float end_angle);
    float get_side_lat_ave(float start, float end);
    size_t rad2index(float rad);

    float angle_min_, angle_max_;
    float angle_increment_;
    float range_min_, range_max_;
    std::vector<float> ranges_;
};
} // namespace ScanData
#endif // SCANDATA__SCANDATA_HPP_