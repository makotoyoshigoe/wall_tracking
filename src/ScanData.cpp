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
    ranges_.resize(msg->ranges.size());
}

ScanData::~ScanData()
{
}

void ScanData::dataUpdate(std::vector<float> ranges)
{
    if(ranges.size() != ranges_.size()) ranges_.resize(ranges.size());
    for(int i=0; i<ranges.size(); ++i) ranges_[i] = ranges[i];
}

float ScanData::frontWallCheck(float start_deg, float threshold)
{
    int start_index = deg2index(start_deg);
    int end_index = deg2index(-start_deg);
    float sum = 0, sum_i = 0;
    for (int i = start_index; i <= end_index; ++i) {
        float range = ranges_[i] * cos(index2rad(i));
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
        float add = (ranges_[i] != INFINITY && ranges_[i] != NAN) ? ranges_[i] * fabsf(sin(index2rad(i))) : range_max_;
        sum += add;
        ++sum_i;
    }
    float per = sum / static_cast<float>(sum_i);
    return per;
}

float ScanData::openPlaceCheck(float start_deg, float end_deg, float threshold)
{
    // RCLCPP_INFO(rclcpp::get_logger("ScanData"), "start: %f, end: %f", start_deg, end_deg);
    float start_index = deg2index(start_deg);
    float end_index = deg2index(end_deg);
    int sum = 0, sum_i = 0;
    for(int i=start_index; i<=end_index; ++i){
        float range = ranges_[i];
        sum += (range < range_min_ || range > threshold || range == INFINITY);
        ++sum_i;
    }
    float per = static_cast<float>(sum) / static_cast<float>(sum_i);
    return per;
}

bool ScanData::conflictCheck(float deg, float threshold)
{
    float rad = DEG2RAD(deg);
    float range = ranges_[deg2index(deg)] * sin(rad);
    if(range  > threshold) return true;
    return false;
}

bool ScanData::thresholdCheck(float deg, float threshold)
{
    int index = deg2index(deg);
    if(ranges_[index] > threshold) return true;
    else false;
}

bool ScanData::noiseCheck(float deg){
  if(deg < range_min_ || std::isnan(deg)) return true;
  return false;
}

int ScanData::deg2index(float deg) { return (deg - angle_min_) / angle_increment_; }

float ScanData::index2deg(int index) { return index * angle_increment_ + angle_min_; }

float ScanData::index2rad(int index) { return index2deg(index) * M_PI / 180; }
} // namespace ScanData