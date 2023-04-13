// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#ifndef WALL_TRACKING__WALL_TRACKING_HPP_
#define WALL_TRACKING__WALL_TRACKING_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

namespace WallTracking {

class WallTracking : public rclcpp::Node {
public:
  WallTracking();

protected:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void set_param();
  void get_param();
  void init_scan_sub();
  void init_cmd_vel_pub();
  void init_variable();
  double pid_control(double input, double goal);
  double compute_lidar_array_ave(std::vector<float> array, int sample, int start_i, double range_max);

private:
  size_t count_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  std::string robot_name;
  std::string cmd_vel_topic_name;
  float distance_from_wall;
  float distance_to_stop;
  float max_linear_vel;
  float max_angular_vel, min_angular_vel;
  float data0;
  float sampling_rate;
  float ei_;
  float kp, ki, kd;
  float lateral_mean;
  int sampling_point, start_range;
  geometry_msgs::msg::Twist cmd_vel_msg;
};

} // namespace WallTracking

#endif // WALL_TRACKING__WALL_TRACKING_HPP_
