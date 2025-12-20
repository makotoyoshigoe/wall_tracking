// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include<wall_tracking_executor/ScanData.hpp>
#include<rclcpp/rclcpp.hpp>

namespace WallTracking{

ScanData::ScanData(void)
{

}

ScanData::~ScanData()
{
}

std::array<float, 2> ScanData::get_minimum_laser(
    float start_angle, float end_angle)
{
    size_t is = rad2index(start_angle), ie = rad2index(end_angle);
    float min_range = range_max_ * 1.1;
    size_t min_i = 0;
    for(size_t i = is; i <= ie; ++i){
        if(min_range > ranges_[i]){
            min_range = ranges_[i];
            min_i = i;
        }
    }
    return {index2rad(min_i), min_range};
}

void ScanData::dataUpdate(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
    angle_min_ = msg->angle_min;
    angle_max_ = msg->angle_max;
    angle_increment_ = msg->angle_increment;
    range_min_ = msg->range_min;
    range_max_ = msg->range_max;
    frame_id_ = msg->header.frame_id;
    if(ranges_.size() != msg->ranges.size()){
        ranges_.clear();
        ranges_.resize(msg->ranges.size());
    }
    float range_min = msg->range_min;
    float range_max = msg->range_max;
    for(size_t i = 0; i < msg->ranges.size(); ++i){
        ranges_[i] = range_min > msg->ranges[i] || msg->ranges[i] > range_max ? 
                      range_max : msg->ranges[i];
    }
}

// [0]: 平均、[1]: 分散
std::array<float, 2> ScanData::get_side_lat_stat(float start, float end)
{
    size_t is = rad2index(start), ie = rad2index(end);
    float angle = start, sum = 0.f, sum_i = 0.f;
    std::vector<float> valid_ranges;
    for(size_t i = is; i <= ie; ++i){
        float r = ranges_[i] < range_min_ || range_max_ < ranges_[i] ? range_max_ : ranges_[i];
        if(!std::isnan(r)){
            float lat = fabs(r * sin(angle));
            valid_ranges.push_back(lat);
            sum += lat;
            sum_i++;
        }
        angle += angle_increment_;
    }
    float mean = sum / sum_i;
    float sq_error = 0.f;
    for(auto & range : valid_ranges) sq_error += pow(range - mean, 2);
    return {mean, sum_i > 1.0f ? sq_error / (sum_i - 1) : 0.0f};
}

float ScanData::index2rad(int index) { return index * angle_increment_ + angle_min_; }

size_t ScanData::rad2index(float rad)
{
    return static_cast<size_t>((rad - angle_min_) / angle_increment_);
}

} // namespace ScanData