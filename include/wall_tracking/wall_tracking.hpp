// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#ifndef WALL_TRACKING__WALL_TRACKING_HPP_
#define WALL_TRACKING__WALL_TRACKING_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <vector>

namespace WallTracking {

class WallTracking : public rclcpp::Node {
public:
  WallTracking();

protected:
  void scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void set_param();
  void get_param();
  void init_scan_sub();
  void init_cmd_vel_pub();
  void init_variable();
  double pid_control(double input);
  double compute_lidar_array_ave(std::vector<float> array, int start_deg,
                                 int end_deg);
  int convert_deg2index(int deg);
  double convert_index2deg(int index);
  void pub_cmd_vel(double linear_x, double anguler_z);

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  geometry_msgs::msg::Twist cmd_vel_msg;
  std::string robot_name;
  std::string cmd_vel_topic_name;
  float distance_from_wall;
  float distance_to_stop;
  float max_linear_vel;
  float max_angular_vel, min_angular_vel;
  float sampling_rate;
  float ei_;
  float kp, ki, kd;
  float lateral_mean;
  int start_deg_lateral, end_deg_lateral;
  int start_deg_longitude, end_deg_longitude;
  float range_max;
  float angle_increment_deg;
  float angle_min_deg;
  int ray_th;
};

} // namespace WallTracking

#endif // WALL_TRACKING__WALL_TRACKING_HPP_
