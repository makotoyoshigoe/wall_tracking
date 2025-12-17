// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "wall_tracking_executor/wall_tracking_node.hpp"

#include <tf2_ros/create_timer_ros.h>

#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2/utils.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace WallTracking{

void WallTracking::init_tf(void)
{
	tf_listener_.reset();
	tf_buffer_.reset();

	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
	auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
	  get_node_base_interface(), get_node_timers_interface(),
	  create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false));
	tf_buffer_->setCreateTimerInterface(timer_interface);
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    init_tf_ = true;
}

std::optional<geometry_msgs::msg::Pose2D> WallTracking::get_parts_pose_in_map(
    std::string frame_id)
{
    geometry_msgs::msg::PoseStamped ident;
	ident.header.frame_id = frame_id;
	ident.header.stamp = rclcpp::Time(0);
	tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

	geometry_msgs::msg::PoseStamped odom_pose;
	try {
		this->tf_buffer_->transform(ident, odom_pose, map_frame_id_);
	} catch (tf2::TransformException & e) {
		RCLCPP_WARN(
		  get_logger(), "Failed to compute parts pose, skipping scan (%s)", e.what());
		return std::nullopt;
	}
    geometry_msgs::msg::Pose2D pose_2d;
	pose_2d.x = odom_pose.pose.position.x;
	pose_2d.y = odom_pose.pose.position.y;
	pose_2d.theta = tf2::getYaw(odom_pose.pose.orientation);

	return pose_2d;
}

}