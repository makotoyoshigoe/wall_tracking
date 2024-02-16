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
    float angle_min_, angle_max_;
    float angle_increment_;
    float range_min_, range_max_;

    std::vector<float> ranges_;
    sensor_msgs::msg::LaserScan::ConstSharedPtr tmp_scan_msg_;
public:
    ScanData(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
    ~ScanData();
    void dataUpdate(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
    float frontWallCheck(float start_deg, float threshold);
    float leftWallCheck(float start_deg, float end_deg);
    void openPlaceCheck(float start_deg, float end_deg, float threshold, float &per, float &mean);
    bool conflictCheck(float deg, float threshold);
    bool thresholdCheck(float deg, float threshold);
    bool noiseCheck(float deg);
    int deg2index(float deg);
    float index2deg(int index);
    float index2rad(int index);
};
} // namespace ScanData
#endif // SCANDATA__SCANDATA_HPP_