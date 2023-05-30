// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#ifndef WALL_TRACKING__WALL_TRACKING_HPP_
#define WALL_TRACKING__WALL_TRACKING_HPP_

#define DEG2RAD(deg) ((deg)*M_PI/180)
#define RAD2DEG(rad) ((rad)*180/M_PI)

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
  double lateral_pid_control(double input);
  double longitude_pid_control(double input);
  double ray_mean(std::vector<float> array, int start_deg,
                                 int end_deg);
  int deg2index(int deg);
  double index2deg(int index);
  double index2rad(int index);
  void pub_cmd_vel(float linear_x, float anguler_z);

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  geometry_msgs::msg::Twist cmd_vel_msg_;
  std::string robot_name_;
  std::string cmd_vel_topic_name_;
  double distance_from_wall_;
  double distance_to_stop_;
  float max_linear_vel_;
  float max_angular_vel_, min_angular_vel_;
  double sampling_rate_;
  float ei_;
  double kp_, ki_, kd_;
  int start_deg_lateral_, end_deg_lateral_;
  float range_max_;
  float angle_increment_deg_;
  float angle_min_deg_;
  int ray_th_;
  float wheel_separation_;
  double distance_to_skip_;
  double flw_deg_;
};

} // namespace WallTracking

#endif // WALL_TRACKING__WALL_TRACKING_HPP_
