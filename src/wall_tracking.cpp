// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "wall_tracking/wall_tracking.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using namespace std::chrono_literals;

namespace WallTracking {
WallTracking::WallTracking() : Node("wall_tracking_node") 
{
    set_param();
    get_param();
    init_variable();
    init_sub();
    init_pub();
    init_action();
}

WallTracking::~WallTracking()
{
}

void WallTracking::set_param() 
{
    this->declare_parameter("max_linear_vel", 0.0);
    this->declare_parameter("max_angular_vel", 0.0);
    this->declare_parameter("min_angular_vel", 0.0);
    this->declare_parameter("distance_from_wall", 0.0);
    this->declare_parameter("distance_to_stop", 0.0);
    this->declare_parameter("sampling_rate", 0.0);
    this->declare_parameter("kp", 0.0);
    this->declare_parameter("ki", 0.0);
    this->declare_parameter("kd", 0.0);
    this->declare_parameter("start_deg_lateral", 0);
    this->declare_parameter("end_deg_lateral", 0);
    this->declare_parameter("stop_ray_th", 0.0);
    this->declare_parameter("wheel_separation", 0.0);
    this->declare_parameter("distance_to_skip", 0.0);
    this->declare_parameter("cmd_vel_topic_name", "");
    this->declare_parameter("open_place_distance", 0.0);
    this->declare_parameter("select_angvel", std::vector<double>(2, 0.0));
    this->declare_parameter("detection_div_deg", std::vector<double>(2, 0.0));
}

void WallTracking::get_param() 
{
    this->get_parameter("max_linear_vel", max_linear_vel_);
    this->get_parameter("max_angular_vel", max_angular_vel_);
    this->get_parameter("min_angular_vel", min_angular_vel_);
    this->get_parameter("distance_from_wall", distance_from_wall_);
    this->get_parameter("distance_to_stop", distance_to_stop_);
    this->get_parameter("sampling_rate", sampling_rate_);
    this->get_parameter("kp", kp_);
    this->get_parameter("ki", ki_);
    this->get_parameter("kd", kd_);
    this->get_parameter("start_deg_lateral", start_deg_lateral_);
    this->get_parameter("end_deg_lateral", end_deg_lateral_);
    this->get_parameter("stop_ray_th", stop_ray_th_);
    this->get_parameter("wheel_separation", wheel_separation_);
    this->get_parameter("distance_to_skip", distance_to_skip_);
    this->get_parameter("cmd_vel_topic_name", cmd_vel_topic_name_);
    this->get_parameter("open_place_distance", open_place_distance_);
    this->get_parameter("detection_div_deg", detection_div_deg_);
    this->get_parameter("select_angvel", select_angvel_);
    // RCLCPP_INFO(this->get_logger(), "%d", detection_div_deg_.size());
}

void WallTracking::init_sub() 
{
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::QoS(10),
        std::bind(&WallTracking::scan_callback, this, std::placeholders::_1));
    gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "gnss/fix", rclcpp::QoS(10),
        std::bind(&WallTracking::gnss_callback, this, std::placeholders::_1));
    wall_tracking_flg_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "wall_tracking_flg", rclcpp::QoS(10),
        std::bind(&WallTracking::wall_tracking_flg_callback, this, std::placeholders::_1));
}

void WallTracking::init_pub() 
{
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_name_, rclcpp::QoS(10));
    open_place_arrived_pub_ = this->create_publisher<std_msgs::msg::Bool>("open_place_arrived", rclcpp::QoS(10));
    open_place_detection_pub_ = this->create_publisher<std_msgs::msg::String>("open_place_detection", rclcpp::QoS(10));
}

void WallTracking::init_action() 
{
    wall_tracking_action_srv_ = rclcpp_action::create_server<WallTrackingAction>(
        this, "wall_tracking",
        std::bind(&WallTracking::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&WallTracking::handle_cancel, this, std::placeholders::_1),
        std::bind(&WallTracking::handle_accepted, this, std::placeholders::_1));
}

void WallTracking::init_variable() 
{
    ei_ = 0.0;
    fwc_deg_ = RAD2DEG(atan2f(-wheel_separation_ / 2, distance_to_stop_));
    float y = distance_from_wall_, x = distance_to_skip_;
    x += distance_from_wall_ / tan(DEG2RAD(start_deg_lateral_));
    flw_deg_ = RAD2DEG(atan2(y, x));
    open_place_ = false;
    outdoor_ = false;
    init_scan_data_ = false;
    vel_open_place_ = max_linear_vel_ / 3;
    cmd_vel_ = max_linear_vel_;
    wall_tracking_flg_ = false;
    pre_e_ = 0.;
}

void WallTracking::pub_cmd_vel(float linear_x, float angular_z) 
{
    cmd_vel_msg_.linear.x = std::min(linear_x, max_linear_vel_);
    cmd_vel_msg_.angular.z = std::max(std::min(angular_z, max_angular_vel_), min_angular_vel_);
    cmd_vel_pub_->publish(cmd_vel_msg_);
}

void WallTracking::scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg) 
{
    if (!init_scan_data_) {
        scan_data_.reset(new ScanData(msg));
        init_scan_data_ = true;
        RCLCPP_INFO(this->get_logger(), "initialized scan data");
    }
    scan_data_->dataUpdate(msg);
    switch (outdoor_)
    {
    case false:
        open_place_ = false;
        break;
    
    case true:
        float per, mean;
        // float p = scan_data_->openPlaceCheck(-90., 90., open_place_distance_, per, mean);
        scan_data_->openPlaceCheck(-90., 90., open_place_distance_, per, mean);
        open_place_ = !open_place_ ? (per >= 0.7) : per >= 0.4;
        cmd_vel_ = !open_place_ ? max_linear_vel_ : vel_open_place_;
    }
    pub_open_place_arrived(open_place_);
    if(wall_tracking_flg_) navigateOpenPlace();
    else pub_cmd_vel(0., 0.);
    // RCLCPP_INFO(this->get_logger(), "update scan data");
}

void WallTracking::gnss_callback(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
    outdoor_ = msg->position_covariance_type == msg->COVARIANCE_TYPE_UNKNOWN ? false : true;
    // RCLCPP_INFO(this->get_logger(), "outdoor: %d", outdoor_);
}

void WallTracking::wall_tracking_flg_callback(std_msgs::msg::Bool::ConstSharedPtr msg)
{
    wall_tracking_flg_ = msg->data;
}

float WallTracking::lateral_pid_control(float input) 
{
    float e = input - distance_from_wall_;
    ei_ += e * sampling_rate_;
    float ed = (e - pre_e_) / sampling_rate_;
    pre_e_ = e;
    return e * kp_ + ei_ * ki_ + ed * kd_;
}

void WallTracking::turn()
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = DEG2RAD(-45);
    cmd_vel_pub_->publish(msg);
    rclcpp::sleep_for(500ms);
}

void WallTracking::wallTracking()
{
    float gap_th = distance_from_wall_;
    bool gap_start = scan_data_->conflictCheck(start_deg_lateral_, gap_th);
    bool gap_end = scan_data_->conflictCheck(90., gap_th);
    bool front_left_wall = scan_data_->thresholdCheck(flw_deg_, 1.91);
    if ((gap_start || gap_end) && !front_left_wall &&
        !scan_data_->noiseCheck(flw_deg_)) {
        pub_cmd_vel(cmd_vel_, 0.0);
        // RCLCPP_INFO(get_logger(), "skip");
    } else {
        double lateral_mean = scan_data_->leftWallCheck(start_deg_lateral_, end_deg_lateral_);
        double angular_z = lateral_pid_control(lateral_mean);
        pub_cmd_vel(cmd_vel_, angular_z);
        // RCLCPP_INFO(get_logger(), "range: %lf", lateral_mean);
    }
}

void WallTracking::navigateOpenPlace() 
{
    float front_wall_check = scan_data_->frontWallCheck(fwc_deg_, distance_to_stop_);
    std::string detection_res = "Indoor";
    switch (outdoor_)
    {
        case false:
            if (front_wall_check >= stop_ray_th_) turn();
            else wallTracking();
        break;
        case true:
            if (front_wall_check >= stop_ray_th_) turn();
            else {
                int div_num = select_angvel_.size(), j = 0;
                std::vector<float> evals(div_num+1, 0.), means(div_num+1, 0.);
                float per, mean;
                for(int i=0; i<detection_div_deg_.size(); i+=2){
                    // float res = scan_data_->openPlaceCheck(detection_div_deg_[i], detection_div_deg_[i+1], open_place_distance_, per, mean);
                    scan_data_->openPlaceCheck(detection_div_deg_[i], detection_div_deg_[i+1], open_place_distance_, per, mean);
                    evals[j] = per < 0.7 ? -1. : per;
                    means[j] = mean;
                    RCLCPP_INFO(this->get_logger(), "Range %d : eval=%lf, mean=%lf", j+1, evals[j], means[j]);
                    ++j;
                }
                auto max_iter = std::max_element(evals.begin(), evals.end());
                int max_index = std::distance(evals.begin(), max_iter);
                if(max_index != div_num){
                    pub_cmd_vel(cmd_vel_, select_angvel_[max_index]);
                    detection_res = "Detect open place";
                } else wallTracking();
                // RCLCPP_INFO(this->get_logger(), "1: %f 2: %f, 3:%f, 4: %f, max i: %d", evals[0], evals[1], evals[2], evals[3], max_index);
            }
        break;
    }
    pub_open_place_detection(detection_res);
}

void WallTracking::pub_open_place_arrived(bool open_place_arrived)
{
    open_place_arrived_msg_.data = open_place_arrived;
    open_place_arrived_pub_->publish(open_place_arrived_msg_);
}

void WallTracking::pub_open_place_detection(std::string open_place_detection)
{
    open_place_detection_msg_.data = open_place_detection;
    open_place_detection_pub_->publish(open_place_detection_msg_);
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
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
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

void WallTracking::execute(
    const std::shared_ptr<GoalHandleWallTracking> goal_handle) 
{
    RCLCPP_INFO(this->get_logger(), "EXECUTE");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<WallTrackingAction::Feedback>();
    auto result = std::make_shared<WallTrackingAction::Result>();
    feedback->end = false;
    wall_tracking_flg_ = true;
    rclcpp::Rate loop_rate(20);
    while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
            wall_tracking_flg_ = false;
            result->get = false;
            goal_handle->canceled(result);
            pub_cmd_vel(0.0, 0.0);
            RCLCPP_INFO(this->get_logger(), "Goal Canceled");
            return;
        }
        loop_rate.sleep();
    }
    if (rclcpp::ok()) {
        result->get = true;
        wall_tracking_flg_ = false;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal Succeded");
    }
}
} // namespace WallTracking
