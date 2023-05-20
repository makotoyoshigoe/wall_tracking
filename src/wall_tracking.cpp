// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "wall_tracking/wall_tracking.hpp"

#include <algorithm>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

using namespace std::chrono_literals;

namespace WallTracking {
WallTracking::WallTracking() : Node("wall_tracking_node") {
  set_param();
  get_param();
  init_variable();
  init_scan_sub();
  init_cmd_vel_pub();
}

void WallTracking::scan_callback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  range_max = msg->range_max;
  angle_increment_deg = msg->angle_increment * 180 / M_PI;
  angle_min_deg = round(msg->angle_min * 180 / M_PI);

  int start_index = convert_deg2index(atan2f(-wheel_separation/2, distance_to_stop)*180/M_PI);
  int end_index = convert_deg2index(atan2f(wheel_separation/2, distance_to_stop)*180/M_PI);
  // RCLCPP_INFO(get_logger(), "start: %d, end: %d", start_index, end_index);
  int ray = 0;
  std::vector<float> ray_ranges, ray_ranges_deg;
  for (int i = start_index; i <= end_index; ++i) {
    float range = msg->ranges[i] * cos(convert_index2deg(i) * (M_PI / 180));
    // std::cout << msg->ranges[i] << ", ";
    if (range > msg->range_min && range < distance_to_stop){
      ray_ranges_deg.push_back(convert_index2deg(i));
      ray_ranges.push_back(range);
      ++ray;
    }
  }
  // std::cout << std::flush;
  rt_sum = (ray >= ray_th) ? rt_sum + 1 : 0;
  if (rt_sum > rt_sum_th) {
    // for(int i=0; i<ray_ranges.size(); ++i){
    //   RCLCPP_INFO(get_logger(), "ray num: %d", ray);
    //   // RCLCPP_INFO(get_logger(), "deg: %lf, range: %lf", ray_ranges_deg[i], ray_ranges[i]);
    // }
    RCLCPP_INFO(get_logger(), "ray num: %d", ray);
    pub_cmd_vel(0.0, -M_PI / 4);
    rclcpp::sleep_for(2000ms);
  } else {
    lateral_mean = compute_lidar_array_ave(msg->ranges, start_deg_lateral,
                                           end_deg_lateral);
    double angular_z = pid_control(lateral_mean);
    pub_cmd_vel(max_linear_vel, angular_z);
  }
}

void WallTracking::set_param() {
  declare_parameter("robot_name", "");
  declare_parameter("max_linear_vel", 0.0);
  declare_parameter("max_angular_vel", 0.0);
  declare_parameter("min_angular_vel", 0.0);
  declare_parameter("distance_from_wall", 0.0);
  declare_parameter("distance_to_stop", 0.0);
  declare_parameter("sampling_rate", 0.0);
  declare_parameter("kp", 0.0);
  declare_parameter("ki", 0.0);
  declare_parameter("kd", 0.0);
  declare_parameter("start_deg_lateral", 0);
  declare_parameter("end_deg_lateral", 0);
  declare_parameter("ray_th", 0);
  declare_parameter("rt_sum_th", 0);
  declare_parameter("wheel_separation", 0.0);
}

void WallTracking::get_param() {
  robot_name = get_parameter("robot_name").as_string();
  max_linear_vel = get_parameter("max_linear_vel").as_double();
  max_angular_vel = get_parameter("max_angular_vel").as_double();
  min_angular_vel = get_parameter("min_angular_vel").as_double();
  distance_from_wall = get_parameter("distance_from_wall").as_double();
  distance_to_stop = get_parameter("distance_to_stop").as_double();
  sampling_rate = get_parameter("sampling_rate").as_double();
  kp = get_parameter("kp").as_double();
  ki = get_parameter("ki").as_double();
  kd = get_parameter("kd").as_double();
  start_deg_lateral = get_parameter("start_deg_lateral").as_int();
  end_deg_lateral = get_parameter("end_deg_lateral").as_int();
  ray_th = get_parameter("ray_th").as_int();
  rt_sum_th = get_parameter("rt_sum_th").as_int();
  wheel_separation = get_parameter("wheel_separation").as_double();
}

void WallTracking::init_scan_sub() {
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::QoS(10),
      std::bind(&WallTracking::scan_callback, this, std::placeholders::_1));
}

void WallTracking::init_cmd_vel_pub() {
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_name, rclcpp::QoS(10));
}

void WallTracking::init_variable() {
  ei_ = 0.0;
  cmd_vel_topic_name = robot_name + "/cmd_vel";
  rt_sum = 0;
  pre_range = 1000000;
}

double WallTracking::pid_control(double input) {
  RCLCPP_INFO(get_logger(), "range: %lf", input);
  double e_range = pre_range - input;
  float e = input - distance_from_wall;
  ei_ += e * sampling_rate;
  float ed = e / sampling_rate;

  float ang_z = e * kp + ei_ * ki + ed * kd;
  if(e < 0) ang_z *= -1;
  ang_z = std::max(std::min(ang_z, max_angular_vel), min_angular_vel);
  pre_range = input;
  return ang_z;
}

double WallTracking::compute_lidar_array_ave(std::vector<float> array,
                                             int start_deg, int end_deg) {
  float sum = 0, sum_i = 0;
  int start_index = convert_deg2index(start_deg);
  int end_index = convert_deg2index(end_deg);
  for (int i = start_index; i <= end_index; ++i) {
    if (array[i] != INFINITY && array[i] != NAN) {
      sum += array[i] * sin(convert_index2deg(i) * (M_PI / 180));
    } else
      sum += range_max;
    ++sum_i;
  }
  return sum / sum_i;
}

void WallTracking::pub_cmd_vel(double linear_x, double angular_z) {
  cmd_vel_msg.linear.x = linear_x;
  cmd_vel_msg.angular.z = angular_z;
  cmd_vel_pub_->publish(cmd_vel_msg);
}

int WallTracking::convert_deg2index(int deg) {
  return (deg - angle_min_deg) / angle_increment_deg;
}

double WallTracking::convert_index2deg(int index) {
  return index * angle_increment_deg + angle_min_deg;
}
} // namespace WallTracking
