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
  range_max = msg->range_max;
  angle_increment_deg = RAD2DEG(msg->angle_increment);
  angle_min_deg = round(RAD2DEG(msg->angle_min));

  int start_index =
      deg2index(RAD2DEG(atan2f(-wheel_separation / 2, distance_to_stop)));
  int end_index =
      deg2index(RAD2DEG(atan2f(wheel_separation / 2, distance_to_stop)));
  int ray = 0;
  float sum = 0, sum_i = 0;
  std::vector<float> ray_ranges, ray_ranges_deg;
  for (int i = start_index; i <= end_index; ++i) {
    float range = msg->ranges[i] * cos(index2rad(i));
    if (range > msg->range_min && range < distance_to_stop) {
      ++ray;
    } else if (range > msg->range_min && range < distance_to_turn) {
      sum += range;
      ++sum_i;
    }
  }
  double lateral_mean =
      ray_mean(msg->ranges, start_deg_lateral, end_deg_lateral);
  // auto max_i = std::max_element(msg->ranges.begin()+deg2index(start_deg_lateral-1), msg->ranges.begin()+deg2index(end_deg_lateral)+1);
  // RCLCPP_INFO(get_logger(), "max ray index: %lf, max ray: %lf", index2deg(*max_i), msg->ranges[*max_i]);
  bool gap = msg->ranges[deg2index(gap_deg)] * sin(DEG2RAD(gap_deg)) >
             distance_from_wall*1.1;
  bool gap90 = msg->ranges[deg2index(end_deg_lateral+1)] > distance_from_wall*1.1;
  bool front_left_wall =
      msg->ranges[deg2index(gap2_deg)] * sin(DEG2RAD(gap2_deg)) <=
      distance_from_wall;

  // RCLCPP_INFO(get_logger(), "front left wall: %lf",
              // msg->ranges[deg2index(gap2_deg)] * sin(DEG2RAD(gap2_deg)));
  // RCLCPP_INFO(get_logger(), "range: %lf", lateral_mean);
  // RCLCPP_INFO(get_logger(), "gap: %d, gap90:%d, flw: %d", gap, gap90, front_left_wall);

  // if(!judge && gap){
  //   if(front_left_wall) skip = true;
  //   else skip = false;
  //   judge = true;
  // }

  // if(judge && !gap && !gap90 && front_left_wall){
  //   skip = false;
  //   judge = false;
  // }
  if (ray >= ray_th) {
    RCLCPP_INFO(get_logger(), "ray num: %d", ray);
    pub_cmd_vel(max_linear_vel / 4, -M_PI / 4);
    rclcpp::sleep_for(2000ms);
  } else if ((gap || gap90) && front_left_wall) {
    pub_cmd_vel(max_linear_vel, -0.0);
    RCLCPP_INFO(get_logger(), "skip");
  // } else if (!(gap && gap90) && (sum / sum_i) < distance_to_turn && sum_i > 5) {
  //   RCLCPP_INFO(get_logger(), "turn");
  //   float k = (sum / sum_i) / distance_to_turn;
  //   pub_cmd_vel(max_linear_vel * k, longitude_pid_control(sum / sum_i));
  } else {
    double angular_z = lateral_pid_control(lateral_mean);
    pub_cmd_vel(max_linear_vel, angular_z);
    RCLCPP_INFO(get_logger(), "range: %lf, ang_z: %lf", lateral_mean, angular_z);
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
  declare_parameter("kp2", 0.0);
  declare_parameter("ki2", 0.0);
  declare_parameter("kd2", 0.0);
  declare_parameter("start_deg_lateral", 0);
  declare_parameter("end_deg_lateral", 0);
  declare_parameter("ray_th", 0);
  declare_parameter("wheel_separation", 0.0);
  declare_parameter("distance_to_turn", 0.0);
  declare_parameter("distance_to_skip", 0.0);
  declare_parameter("gap_deg", 0);
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
  kp2 = get_parameter("kp2").as_double();
  ki2 = get_parameter("ki2").as_double();
  kd2 = get_parameter("kd2").as_double();
  start_deg_lateral = get_parameter("start_deg_lateral").as_int();
  end_deg_lateral = get_parameter("end_deg_lateral").as_int();
  ray_th = get_parameter("ray_th").as_int();
  wheel_separation = get_parameter("wheel_separation").as_double();
  distance_to_turn = get_parameter("distance_to_turn").as_double();
  distance_to_skip = get_parameter("distance_to_skip").as_double();
  gap_deg = get_parameter("gap_deg").as_int();
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
  ei2_ = 0.0;
  cmd_vel_topic_name = robot_name + "/cmd_vel";
  gap2_deg = RAD2DEG(atan2(distance_from_wall,
                           distance_to_skip + distance_from_wall *
                                                  tan(DEG2RAD(90 - gap_deg))));
  skip = false;
  judge = false;
  // RCLCPP_INFO(get_logger(), "gap2: %lf", gap2_deg);
}

double WallTracking::lateral_pid_control(double input) {
  float e = input - distance_from_wall;
  ei_ += e * sampling_rate;
  float ed = e / sampling_rate;

  float ang_z = e * kp + ei_ * ki + ed * kd;
  return ang_z;
}

double WallTracking::longitude_pid_control(double input) {
  float e = input - distance_to_turn;
  ei2_ += e * sampling_rate;
  float ed = e / sampling_rate;

  float ang_z = e * kp2 + ei2_ * ki2 + ed * kd2;
  return ang_z;
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
      sum += range_max;
    ++sum_i;
  }
  return sum / sum_i;
}

void WallTracking::pub_cmd_vel(float linear_x, float angular_z) {
  cmd_vel_msg.linear.x = std::min(linear_x, max_linear_vel);
  cmd_vel_msg.angular.z =
      std::max(std::min(angular_z, max_angular_vel), min_angular_vel);
  cmd_vel_pub_->publish(cmd_vel_msg);
}

int WallTracking::deg2index(int deg) {
  return (deg - angle_min_deg) / angle_increment_deg;
}

double WallTracking::index2deg(int index) {
  return index * angle_increment_deg + angle_min_deg;
}

double WallTracking::index2rad(int index) {
  return index2deg(index) * M_PI / 180;
}
} // namespace WallTracking
