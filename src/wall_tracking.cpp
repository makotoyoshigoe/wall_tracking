// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "wall_tracking/wall_tracking.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

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
  range_max_ = msg->range_max;
  angle_increment_deg_ = RAD2DEG(msg->angle_increment);
  angle_min_deg_ = round(RAD2DEG(msg->angle_min));

  int start_index =
      deg2index(RAD2DEG(atan2f(-wheel_separation_ / 2, distance_to_stop_)));
  int end_index = -start_index;
  int ray = 0;
  float sum = 0, sum_i = 0;
  for (int i = start_index; i <= end_index; ++i) {
    float range = msg->ranges[i] * cos(index2rad(i));
    if (range > msg->range_min && range < distance_to_stop_) {
      ++ray;
    }
  }
  double lateral_mean =
      ray_mean(msg->ranges, start_deg_lateral_, end_deg_lateral_);
  bool gap_start = msg->ranges[deg2index(start_deg_lateral_)] * sin(DEG2RAD(start_deg_lateral_)) >
             distance_from_wall_*1.1;
  bool gap_end = msg->ranges[deg2index(end_deg_lateral_)] * sin(DEG2RAD(end_deg_lateral_)) > distance_from_wall_*1.1;
  bool front_left_wall =
      msg->ranges[deg2index(flw_deg_)] * sin(DEG2RAD(flw_deg_)) <=
      distance_from_wall_;

  if (ray >= ray_th_) {
    RCLCPP_INFO(get_logger(), "ray num: %d", ray);
    pub_cmd_vel(max_linear_vel_ / 4, -M_PI / 4);
    rclcpp::sleep_for(2000ms);
  } else if ((gap_start || gap_end) && front_left_wall) {
    pub_cmd_vel(max_linear_vel_, 0.0);
    RCLCPP_INFO(get_logger(), "skip");
  } else {
    double angular_z = lateral_pid_control(lateral_mean);
    pub_cmd_vel(max_linear_vel_, angular_z);
    RCLCPP_INFO(get_logger(), "range: %lf", lateral_mean);
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
  declare_parameter("wheel_separation", 0.0);
  declare_parameter("distance_to_skip", 0.0);
}

void WallTracking::get_param() {
  robot_name_ = get_parameter("robot_name").as_string();
  max_linear_vel_ = get_parameter("max_linear_vel").as_double();
  max_angular_vel_ = get_parameter("max_angular_vel").as_double();
  min_angular_vel_ = get_parameter("min_angular_vel").as_double();
  distance_from_wall_ = get_parameter("distance_from_wall").as_double();
  distance_to_stop_ = get_parameter("distance_to_stop").as_double();
  sampling_rate_ = get_parameter("sampling_rate").as_double();
  kp_ = get_parameter("kp").as_double();
  ki_ = get_parameter("ki").as_double();
  kd_ = get_parameter("kd").as_double();
  start_deg_lateral_ = get_parameter("start_deg_lateral").as_int();
  end_deg_lateral_ = get_parameter("end_deg_lateral").as_int();
  ray_th_ = get_parameter("ray_th").as_int();
  wheel_separation_ = get_parameter("wheel_separation").as_double();
  distance_to_skip_ = get_parameter("distance_to_skip").as_double();
}

void WallTracking::init_scan_sub() {
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::QoS(10),
      std::bind(&WallTracking::scan_callback, this, std::placeholders::_1));
}

void WallTracking::init_cmd_vel_pub() {
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_name_, rclcpp::QoS(10));
}

void WallTracking::init_variable() {
  ei_ = 0.0;
  cmd_vel_topic_name_ = robot_name_ + "/cmd_vel";
  flw_deg_ = RAD2DEG(atan2(distance_from_wall_,
                           distance_to_skip_ + distance_from_wall_ /
                                                  tan(DEG2RAD(start_deg_lateral_))));
  RCLCPP_INFO(get_logger(), "deg: %lf", flw_deg_);
}

double WallTracking::lateral_pid_control(double input) {
  double e = input - distance_from_wall_;
  ei_ += e * sampling_rate_;
  double ed = e / sampling_rate_;
  return e * kp_ + ei_ * ki_ + ed * kd_;
}

double WallTracking::ray_mean(std::vector<float> array, int start_deg,
                              int end_deg) {
  float sum = 0, sum_i = 0;
  int start_index = deg2index(start_deg);
  int end_index = deg2index(end_deg);
  for (int i = start_index; i <= end_index; ++i) {
    if (array[i] != INFINITY && array[i] != NAN) {
      sum += array[i] * sin(index2rad(i));
    } else
      sum += range_max_;
    ++sum_i;
  }
  return sum / sum_i;
}

void WallTracking::pub_cmd_vel(float linear_x, float angular_z) {
  cmd_vel_msg_.linear.x = std::min(linear_x, max_linear_vel_);
  cmd_vel_msg_.angular.z =
      std::max(std::min(angular_z, max_angular_vel_), min_angular_vel_);
  cmd_vel_pub_->publish(cmd_vel_msg_);
}

int WallTracking::deg2index(int deg) {
  return (deg - angle_min_deg_) / angle_increment_deg_;
}

double WallTracking::index2deg(int index) {
  return index * angle_increment_deg_ + angle_min_deg_;
}

double WallTracking::index2rad(int index) {
  return index2deg(index) * M_PI / 180;
}
} // namespace WallTracking
