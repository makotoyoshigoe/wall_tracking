// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "wall_tracking/wall_tracking.hpp"
#include <atomic>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace WallTracking {
WallTracking::WallTracking() : Node("wall_tracking_node"), count_(0) {
  set_param();
  get_param();
  init_variable();
  init_scan_sub();
  init_cmd_vel_pub();
}

void WallTracking::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  float angle_min_deg = round(msg->angle_min * 180/M_PI);
  float angle_max_deg = round(msg->angle_max * 180/M_PI);
  float angle_increment_deg = msg->angle_increment * 180/M_PI;
  int start_index = (start_deg - angle_min_deg) / angle_increment_deg;
  int end_index = (end_deg - angle_min_deg) /angle_increment_deg;

  RCLCPP_INFO(get_logger(), "start index: %d, end index: %d", start_index, end_index);
  data0 = msg->ranges[(0-angle_min_deg) / angle_increment_deg];

  double sum = 0, sum_i = 0;
  for (int i = start_index; i <= end_index; i++) {
      if (msg->ranges[i] != INFINITY && msg->ranges[i] != NAN) {
        sum += msg->ranges[i] * sin((start_deg + angle_increment_deg * sum_i) * (M_PI / 180));
      } else {
        sum += msg->range_max;
      }
      ++sum_i;
  }

  lateral_mean = sum / sum_i;
  // if (data0 < distance_to_stop) {
  //   cmd_vel_msg.linear.x = 0.0;
  //   cmd_vel_msg.angular.z = -M_PI / 4;
  //   cmd_vel_pub_->publish(cmd_vel_msg);
  //   rclcpp::sleep_for(2000ms);
  // } else {

  // }
  kp = get_parameter("kp").as_double();
  ki = get_parameter("ki").as_double();
  kd = get_parameter("kd").as_double();

  cmd_vel_msg.linear.x = max_linear_vel;
  RCLCPP_INFO(get_logger(), "range: %f", lateral_mean);

  cmd_vel_msg.angular.z = pid_control(lateral_mean, distance_from_wall);
  cmd_vel_pub_->publish(cmd_vel_msg);
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
  declare_parameter("start_deg", 0);
  declare_parameter("end_deg", 0);
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
  start_deg = get_parameter("start_deg").as_int();
  end_deg = get_parameter("end_deg").as_int();
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
}

double WallTracking::pid_control(double input, double goal) {
  float e = input - goal;
  ei_ += e * sampling_rate;
  float ed = e / sampling_rate;

  float ang_z = -1 * (e * kp + ei_ * ki + ed * kd);
  ang_z = std::max(std::min(ang_z, max_angular_vel), min_angular_vel);
  return ang_z;
}

double WallTracking::compute_lidar_array_ave(std::vector<float> array,
                                             int start_i, int end_i,
                                             double range_max) {
  double sum = 0, sum_i = 0;
  for (int i = start_i; i <= end_i; i++) {
    if (array[i] != INFINITY && array[i] != NAN) {
      sum += array[i] * sin((i * 0.3) * (M_PI / 180));
    } else {
      sum += range_max;
    }
    ++sum_i;
  }
  return sum / sum_i;
}
} // namespace WallTracking
