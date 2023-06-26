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
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <tf2_msgs/msg/tf_message.hpp>
#include <string>
#include <vector>

namespace WallTracking {

class WallTracking : public rclcpp::Node {
public:
  WallTracking();

protected:
  void set_param();
  void get_param();
  void init_sub();
  void init_pub();
  void init_variable();
  double lateral_pid_control(double input);
  double longitude_pid_control(double input);
  double ray_mean(std::vector<float> array, int start_deg,
                                 int end_deg);
  int deg2index(int deg);
  double index2deg(int index);
  double index2rad(int index);
  void pub_cmd_vel(float linear_x, float anguler_z);
  void scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void gnss_callback(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  double ray_th_processing(std::vector<float> array, double start, double end);

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  geometry_msgs::msg::Twist cmd_vel_msg_;
  sensor_msgs::msg::NavSatFix nav_sat_fix_msg_;
  nav_msgs::msg::Odometry odom_msg_;
  std::string cmd_vel_topic_name_;
  double distance_from_wall_;
  double distance_to_stop_;
  float max_linear_vel_;
  float max_angular_vel_, min_angular_vel_;
  double sampling_rate_;
  float ei_;
  double kp_, ki_, kd_;
  int start_deg_lateral_, end_deg_lateral_;
  float range_max_, range_min_;
  float angle_increment_deg_;
  float angle_min_deg_;
  int ray_th_;
  float wheel_separation_;
  double distance_to_skip_;
  double flw_deg_;
  double covariance_th_;
  bool open_place_;
};

} // namespace WallTracking

#endif // WALL_TRACKING__WALL_TRACKING_HPP_
