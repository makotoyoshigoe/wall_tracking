// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#ifndef WALL_TRACKING__WALL_TRACKING_HPP_
#define WALL_TRACKING__WALL_TRACKING_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>
#include "wall_tracking_msgs/action/wall_tracking.hpp"
#include "wall_tracking_executor/ScanData.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <wall_tracking_msgs/msg/behavior_stamped_array.hpp>
#include <wall_tracking_msgs/msg/behavior_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

using WallTrackingAction = wall_tracking_msgs::action::WallTracking;
using GoalHandleWallTracking = rclcpp_action::ServerGoalHandle<WallTrackingAction>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

namespace WallTracking {

class WallTracking : public rclcpp::Node {
public:
	WallTracking();
	~WallTracking();

protected:
	void set_param();
	void get_param();
	void init_sub();
	void init_pub();
	void init_action();
	void init_variable();
	float lateral_pid_control(float input);
	void turn();
	void wallTracking();
	void pub_cmd_vel(float linear_x, float anguler_z);
	void scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
	void gnss_callback(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
	void navigateOpenPlace();
	void pub_open_place_arrived(bool open_place_arrived);
	void pub_open_place_detection(std::string open_place_detection);
	void gnss_pose_with_covariance_callback(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
	void goal_pose_callback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
	void alpha_callback(std_msgs::msg::Float32::ConstSharedPtr msg);

	// For wall_tracking action server
	rclcpp_action::GoalResponse handle_goal(
		[[maybe_unused]] const rclcpp_action::GoalUUID & uuid, 
		[[maybe_unused]] std::shared_ptr<const WallTrackingAction::Goal> goal
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

	// For navigate_to_pose action client
	void goalResponceCallback(const std::shared_ptr<GoalHandleNavigateToPose> &goal_handle);
	void feedbackCallback([[maybe_unused]] typename std::shared_ptr<GoalHandleNavigateToPose>, 
						[[maybe_unused]] const std::shared_ptr<const typename NavigateToPose::Feedback> feedback);
	void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result);
	void addBehaviorStamedArray(std::string behavior_name);
	void behaviorStampedPub(void);

private:
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr open_place_arrived_pub_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr open_place_detection_pub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_with_covariance_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr alpha_sub_;

	rclcpp_action::Server<WallTrackingAction>::SharedPtr wall_tracking_action_srv_;

	geometry_msgs::msg::Twist cmd_vel_msg_;
	std_msgs::msg::Bool open_place_arrived_msg_; 
	std_msgs::msg::String open_place_detection_msg_;
	nav2_msgs::action::NavigateToPose::Goal nav_goal_msgs_;

	rclcpp_action::Client<NavigateToPose>::SendGoalOptions nav_send_goal_options_;
	rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_action_client_;

	std::vector<wall_tracking_msgs::msg::BehaviorStamped> behavior_stamped_array_;
	rclcpp::Publisher<wall_tracking_msgs::msg::BehaviorStamped>::SharedPtr behavior_stamped_array_pub_;

	float distance_from_wall_;
	float distance_to_stop_;
	float max_linear_vel_;
	float max_angular_vel_, min_angular_vel_;
	float sampling_rate_;
	float ei_;
	float kp_, ki_, kd_;
	int start_deg_lateral_, end_deg_lateral_;
	float stop_ray_th_;
	float wheel_separation_;
	float distance_to_skip_;
	float flw_deg_;
	bool open_place_;
	float open_place_distance_; 
	bool outdoor_; //屋外にいるかのフラグ
	bool init_scan_data_;
	std::shared_ptr<ScanData> scan_data_;
	float fwc_deg_; //前方の壁との距離をチェックする際に使用するレーザーの開始角度と終了角度
	float vel_open_place_, cmd_vel_;
	bool wall_tracking_flg_;
	bool open_place_linear_;
	std::vector<double> select_angvel_;
	std::vector<double> detection_div_deg_;
	float pre_e_;
	bool gnss_nan_;
	bool recieved_nav_goal_;
	bool only_navigate_open_place_flg_ = false;
	float alpha_;
};

} // namespace WallTracking

#endif // WALL_TRACKING__WALL_TRACKING_HPP_
