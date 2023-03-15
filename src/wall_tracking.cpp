// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "wall_tracking/wall_tracking.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace WallTracking
{
void WallTracking::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  data0 = msg->ranges[0];
  data90 = msg->ranges[90];
  if(data0 < distance_to_stop){
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.angular.z = -1.57/2;
    cmd_vel_pub_->publish(cmd_vel_msg);
    rclcpp::sleep_for(2000ms);
  }else{
    cmd_vel_msg.linear.x = max_linear_vel;
  
    double sum = 0, n = 10, start = 69;
    for (size_t i = 0; i < n; i++) {
      if (msg->ranges[start + i] != INFINITY && msg->ranges[start + i] != NAN) {
        sum += msg->ranges[start + i] * sin((start + i) * (M_PI / 180));
      } else {
        sum += msg->range_max;
      }
    }

    e = sum / n - distance_from_wall;
    ei += e * sampling_rate;
    ed = e / sampling_rate;

    float ang_z = e * kp + ei * ki + ed * kd;

    RCLCPP_INFO(get_logger(), "range: %f", sum / n);

    if (ang_z > max_angular_vel) {ang_z = max_angular_vel;}
    if (ang_z < min_angular_vel) {ang_z = min_angular_vel;}
    cmd_vel_msg.angular.z = ang_z;

    cmd_vel_pub_->publish(cmd_vel_msg);
  }
}

WallTracking::WallTracking()
: Node("wall_tracking_node"), count_(0)
{
  set_param();
  get_param();
  init_variable();
  init_scan_sub();
  init_cmd_vel_pub();
}

void WallTracking::set_param()
{
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
}

void WallTracking::get_param()
{
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
}

void WallTracking::init_scan_sub()
{
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::QoS(
      10), std::bind(&WallTracking::scan_callback, this, std::placeholders::_1));
}

void WallTracking::init_cmd_vel_pub()
{
  cmd_vel_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_name, rclcpp::QoS(10));
}

void WallTracking::init_variable()
{
  e = 0.0;
  ei = 0.0;
  ed = 0.0;
  cmd_vel_topic_name = robot_name + "/cmd_vel";
}
}  // namespace WallTracking
