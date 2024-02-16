// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include<wall_tracking/ScanData.hpp>
#include<rclcpp/rclcpp.hpp>

namespace WallTracking{
ScanData::ScanData(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
    angle_min_ = RAD2DEG(msg->angle_min);
    angle_max_ = RAD2DEG(msg->angle_max);
    angle_increment_ = RAD2DEG(msg->angle_increment);
    range_max_ = msg->range_max;
    range_min_ = msg->range_min;
    // tmp_scan_msg_->ranges.resize(msg->ranges.size());
}

ScanData::~ScanData()
{
}

// void ScanData::dataUpdate(std::vector<float> ranges)
// {
//     if(ranges.size() != tmp_scan_msg_->ranges.size()) tmp_scan_msg_->ranges.resize(ranges.size());
//     for(int i=0; i<ranges.size(); ++i) tmp_scan_msg_->ranges[i] = ranges[i];
// }

void ScanData::dataUpdate(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
    // if(ranges.size() != tmp_scan_msg_->ranges.size()) tmp_scan_msg_->ranges.resize(ranges.size());
    // for(int i=0; i<ranges.size(); ++i) tmp_scan_msg_->ranges[i] = ranges[i];
    tmp_scan_msg_ = msg;
}

float ScanData::frontWallCheck(float start_deg, float threshold)
{
    int start_index = deg2index(start_deg);
    int end_index = deg2index(-start_deg);
    float sum = 0, sum_i = 0;
    for (int i = start_index; i <= end_index; ++i) {
        float range = tmp_scan_msg_->ranges[i] * cos(index2rad(i));
        sum += (range > range_min_ && range < threshold);
        ++sum_i;
    }
    float per = static_cast<float>(sum) / static_cast<float>(sum_i);
    return per;
}

float ScanData::leftWallCheck(float start_deg, float end_deg)
{
    float sum = 0; 
    int sum_i = 0;
    int start_index = deg2index(start_deg);
    int end_index = deg2index(end_deg);
    for (int i = start_index; i <= end_index; ++i) {
        float add = (tmp_scan_msg_->ranges[i] != INFINITY && tmp_scan_msg_->ranges[i] != NAN) ? tmp_scan_msg_->ranges[i] * fabsf(sin(index2rad(i))) : range_max_;
        sum += add;
        ++sum_i;
    }
    float per = sum / static_cast<float>(sum_i);
    return per;
}

void ScanData::openPlaceCheck(float start_deg, float end_deg, float threshold, float &per, float &mean_l)
{
    // RCLCPP_INFO(rclcpp::get_logger("ScanData"), "start: %f, end: %f", start_deg, end_deg);
    float start_index = deg2index(start_deg);
    float end_index = deg2index(end_deg);
    int sum = 0, sum_i = 0, sum_n = 0.;
    float sum_l = 0.;
    for(int i=start_index; i<=end_index; ++i){
        float range = tmp_scan_msg_->ranges[i];
        sum += (range < range_min_ || range >= threshold || range == INFINITY);
        if(range >= threshold){
            sum_l += range;
            ++sum_n;
        }
        ++sum_i;
    }
    per = static_cast<float>(sum) / static_cast<float>(sum_i);
    mean_l = sum_l / static_cast<float>(sum_n);
}

bool ScanData::conflictCheck(float deg, float threshold)
{
    float rad = DEG2RAD(deg);
    float range = tmp_scan_msg_->ranges[deg2index(deg)] * sin(rad);
    if(range  > threshold) return true;
    return false;
}

bool ScanData::thresholdCheck(float deg, float threshold)
{
    int index = deg2index(deg);
    if(tmp_scan_msg_->ranges[index] > threshold) return true;
    else return false;
}

bool ScanData::noiseCheck(float deg){
    int index = deg2index(deg);
    if(tmp_scan_msg_->ranges[index] < range_min_ || std::isnan(tmp_scan_msg_->ranges[index])) return true;
    return false;
}

int ScanData::deg2index(float deg) { return (deg - angle_min_) / angle_increment_; }

float ScanData::index2deg(int index) { return index * angle_increment_ + angle_min_; }

float ScanData::index2rad(int index) { return index2deg(index) * M_PI / 180; }
} // namespace ScanData