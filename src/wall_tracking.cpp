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
  init_sub();
  init_pub();
}

void WallTracking::set_param() {
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
  declare_parameter("cmd_vel_topic_name", "");
  declare_parameter("covariance_th", 0.0);
}

void WallTracking::get_param() {
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
  cmd_vel_topic_name_ = get_parameter("cmd_vel_topic_name").as_string();
  covariance_th_ = get_parameter("covariance_th").as_double();
}

void WallTracking::init_sub() {
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::QoS(10),
      std::bind(&WallTracking::scan_callback, this, std::placeholders::_1));
  gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "gnss/fix", rclcpp::QoS(10),
      std::bind(&WallTracking::gnss_callback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(10),
      std::bind(&WallTracking::odom_callback, this, std::placeholders::_1));
}

void WallTracking::init_pub() {
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_name_, rclcpp::QoS(10));
}

void WallTracking::init_variable() {
  ei_ = 0.0;
  flw_deg_ =
      RAD2DEG(atan2(distance_from_wall_,
                    distance_to_skip_ + distance_from_wall_ /
                                            tan(DEG2RAD(start_deg_lateral_))));
}

double WallTracking::lateral_pid_control(double input) {
  double e = input - distance_from_wall_;
  ei_ += e * sampling_rate_;
  double ed = e / sampling_rate_;
  return e * kp_ + ei_ * ki_ + ed * kd_;
}

float WallTracking::ray_mean(std::vector<float> array, int start_deg,
                              int end_deg) {
  float sum = 0, sum_i = 0;
  int start_index = deg2index(start_deg);
  int end_index = deg2index(end_deg);
  for (int i = start_index; i <= end_index; ++i) {
    if (array[i] != INFINITY && array[i] != NAN) {
      sum += array[i] * sin(index2rad(i));
    } else {
      sum += range_max_;
    }
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

void WallTracking::scan_callback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  range_max_ = msg->range_max;
  range_min_ = msg->range_min;
  angle_increment_deg_ = RAD2DEG(msg->angle_increment);
  angle_min_deg_ = round(RAD2DEG(msg->angle_min));
  int start_index =
      deg2index(RAD2DEG(atan2f(-wheel_separation_ / 2, distance_to_stop_)));
  int end_index = deg2index(RAD2DEG(atan2f(wheel_separation_ / 2, distance_to_stop_)));
  int fw_ray = 0;
  float sum = 0, sum_i = 0;
  for (int i = start_index; i <= end_index; ++i) {
    float range = msg->ranges[i] * cos(index2rad(i));
    if (range > msg->range_min && range < distance_to_stop_) {
      ++fw_ray;
    }
  }
  bool detect_open_place = ray_th_processing(msg->ranges, -5.0, 5.0) >= 0.7;

  bool open_place = ray_th_processing(msg->ranges, -100.0, 0.0) >= 0.7 && ray_th_processing(msg->ranges, 0.0, 100.0) >= 0.7;

  double lateral_mean =
      ray_mean(msg->ranges, start_deg_lateral_, end_deg_lateral_);
  bool gap_start = msg->ranges[deg2index(start_deg_lateral_)] *
                       sin(DEG2RAD(start_deg_lateral_)) >
                   distance_from_wall_ * 2.0;
  bool gap_end = msg->ranges[deg2index(90)] *
                     sin(DEG2RAD(90)) >
                 distance_from_wall_ * 2.0;
  bool front_left_wall_lateral =
      msg->ranges[deg2index(flw_deg_)] * sin(DEG2RAD(flw_deg_)) <=
      distance_from_wall_;
  bool front_left_wall_longitude = msg->ranges[deg2index(flw_deg_)] * cos(DEG2RAD(flw_deg_)) <=
      distance_to_skip_;
  bool front_left_wall = msg->ranges[deg2index(flw_deg_)] <= 1.87;
  // RCLCPP_INFO(get_logger(), "deg: %lf, lw: %lf, long: %lf", flw_deg_, msg->ranges[deg2index(flw_deg_)], msg->ranges[deg2index(flw_deg_)] * cos(DEG2RAD(flw_deg_)));
  if (fw_ray >= ray_th_) {
    RCLCPP_INFO(get_logger(), "fw_ray num: %d", fw_ray);
    pub_cmd_vel(max_linear_vel_ / 4, -M_PI / 6);
    rclcpp::sleep_for(2000ms);
  } else if(open_place){
    pub_cmd_vel(0.0, 0.0);
    rclcpp::sleep_for(2000ms);
    RCLCPP_INFO(get_logger(), "open place");
  } else if(detect_open_place){
    RCLCPP_INFO(get_logger(), "detect open place");
    std::vector<float> direction = {
      ray_mean(msg->ranges, -15, -5), 
      ray_mean(msg->ranges, -5, 5),
      ray_mean(msg->ranges, 5, 15)
    };
    double max = search_max(direction);
    double ang_vel;
    RCLCPP_INFO(get_logger(), "max: %lf, 1: %lf, 2: %lf, 3: %lf", max, direction[0], direction[1], direction[2]);
    if(max == direction[1]) ang_vel = 0.0;
    else if(max == direction[0]) ang_vel = -0.17;
    else if(max == direction[2]) ang_vel = 0.17;
    pub_cmd_vel(max_linear_vel_, 0.0);
    // RCLCPP_INFO(get_logger(), "ang vel: %lf", ang_vel);
  }else if ((gap_start || gap_end) && front_left_wall && !noise(msg->ranges[deg2index(flw_deg_)])) {
    pub_cmd_vel(max_linear_vel_, 0.0);
    RCLCPP_INFO(get_logger(), "skip");
  } else if(open_place){
    pub_cmd_vel(0.0, 0.0);
    RCLCPP_INFO(get_logger(), "open place");
  }else {
    double angular_z = lateral_pid_control(lateral_mean);
    pub_cmd_vel(max_linear_vel_, angular_z);
    RCLCPP_INFO(get_logger(), "range: %lf", lateral_mean);
  }
}

void WallTracking::gnss_callback(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg){
  nav_sat_fix_msg_ = *msg;
  open_place_ = msg->position_covariance[0] > covariance_th_;
}

void WallTracking::odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg){
  odom_msg_ = *msg;
  geometry_msgs::msg::Quaternion q = msg->pose.pose.orientation;
  double yaw = quaternion2euler_yaw(q);
  // RCLCPP_INFO(get_logger(), "euler Z: %lf", yaw);
}

double WallTracking::ray_th_processing(std::vector<float> array, double start, double end){
  double open_place_ray = 0, ray_num = 0;
  for(int i=deg2index(start); i<=deg2index(end); ++i){
    float range = array[i] * cos(index2rad(i));
    if (range < range_min_ || range > range_max_ * 0.5){
      ++open_place_ray;
    }
    ++ray_num;
  }
  double per = open_place_ray / ray_num;
  // RCLCPP_INFO(get_logger(), "o: %lf, r: %lf, o/r: %lf", open_place_ray, ray_num, per);
  return per;
}

double WallTracking::quaternion2euler_yaw(geometry_msgs::msg::Quaternion msg){
  tf2::Quaternion q(
    msg.x, msg.y, msg.z, msg.w
  );
  tf2::Matrix3x3 rpy(q);
  double roll, pitch, yaw;
  rpy.getRPY(roll, pitch, yaw);
  return RAD2DEG(yaw);
}

bool WallTracking::noise(float data){
  if(data < range_min_) return true;
  return false;
}

float WallTracking::search_max(std::vector<float> array){
  double max;
  for(int i=0; i<array.size(); ++i){
    for(int j=i+1; j<array.size(); ++j){
      max = std::max(array[i], array[j]);
    }
  }
  return max;
}
} // namespace WallTracking
