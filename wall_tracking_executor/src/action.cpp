// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "wall_tracking_executor/wall_tracking_node.hpp"

namespace WallTracking{

void WallTracking::goalResponceCallback(const std::shared_ptr<GoalHandleNavigateToPose> & goal_handle)
{
    if(!goal_handle){
        RCLCPP_INFO(this->get_logger(), "Goal was rejected by server");
    }else{
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void WallTracking::feedbackCallback([[maybe_unused]] typename std::shared_ptr<GoalHandleNavigateToPose>, 
        			[[maybe_unused]] const std::shared_ptr<const typename NavigateToPose::Feedback> feedback){}

void WallTracking::resultCallback(const GoalHandleNavigateToPose::WrappedResult & result){
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        recieved_nav_goal_ = false;
        RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
        addBehaviorStamedArray("Navigation Succeeded", false);
        behaviorStampedPub();
        // RCLCPP_INFO(this->get_logger(), "Recieved nav goal: %d", recieved_nav_goal_);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(this->get_logger(), "Goal was aborted");
        addBehaviorStamedArray("Navigation Aborted", false);
        behaviorStampedPub();
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal was canceled");
        addBehaviorStamedArray("Navigation canceled", false);
        behaviorStampedPub();
        // recieved_nav_goal_ = true;
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
}

rclcpp_action::GoalResponse WallTracking::handle_goal(
    [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
    [[maybe_unused]] std::shared_ptr<const WallTrackingAction::Goal> goal) 
{
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WallTracking::handle_cancel(
    const std::shared_ptr<GoalHandleWallTracking> goal_handle) 
{
    RCLCPP_INFO(this->get_logger(), "Wall tracking: Received request to cancel goal");
    pub_cmd_vel(0.0, 0.0);
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void WallTracking::handle_accepted(
    const std::shared_ptr<GoalHandleWallTracking> goal_handle)
{
    std::thread{
    std::bind(&WallTracking::execute, this, std::placeholders::_1),
        goal_handle
    }.detach();
}

}

