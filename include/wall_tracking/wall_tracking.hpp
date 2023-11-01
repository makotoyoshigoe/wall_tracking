// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#ifndef WALL_TRACKING__WALL_TRACKING_HPP_
#define WALL_TRACKING__WALL_TRACKING_HPP_

#define DEG2RAD(deg) ((deg)*M_PI/180)
#define RAD2DEG(rad) ((rad)*180/M_PI)
#define SQUARE(n) (n*n)

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>
#include <vector>
#include <tf2/convert.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "wall_tracking_action/action/wall_tracking.hpp"

using WallTrackingAction = wall_tracking_action::action::WallTracking;
using GoalHandleWallTracking = rclcpp_action::ServerGoalHandle<WallTrackingAction>;

namespace WallTracking {

class WallTracking : public rclcpp::Node {
public:
  WallTracking();

protected:
  void set_param();
  void get_param();
  void init_sub();
  void init_pub();
  void init_action();
  void init_variable();
  double lateral_pid_control(double input);
  double longitude_pid_control(double input);
  float ray_mean(std::vector<double> array, int start_deg,
                                 int end_deg);
  int deg2index(int deg);
  double index2deg(int index);
  double index2rad(int index);
  void pub_cmd_vel(float linear_x, float anguler_z);
  void scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void gnss_callback(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  double ray_th_processing(std::vector<double> array, double start, double end);
  double quaternion2euler_yaw(geometry_msgs::msg::Quaternion msg);
  bool noise(float data);
  float search_max(std::vector<double> array);
  void wallTracking();

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const WallTrackingAction::Goal> goal
  );

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleWallTracking> goal_handle
  );

  void handle_accepted(
    const std::shared_ptr<GoalHandleWallTracking> goal_handle
  );

  void execute(
    const std::shared_ptr<GoalHandleWallTracking> goal_handle
  );

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp_action::Server<WallTrackingAction>::SharedPtr wall_tracking_action_srv_;

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
  double open_place_distance_;
  std::vector<double> ranges_;
};

} // namespace WallTracking

#endif // WALL_TRACKING__WALL_TRACKING_HPP_
