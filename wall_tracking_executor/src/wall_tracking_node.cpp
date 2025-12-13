// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "wall_tracking_executor/wall_tracking_node.hpp"

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
    declare_parameter("max_linear_vel", 0.0);
    declare_parameter("max_angular_vel", 0.0);
    declare_parameter("min_angular_vel", 0.0);
    declare_parameter("distance_from_wall", 0.0);
    declare_parameter("distance_to_stop", 0.0);
    declare_parameter("sampling_rate", 0.0);
    declare_parameter("kp", 0.0);
    declare_parameter("ki", 0.0);
    declare_parameter("kd", 0.0);
    declare_parameter("start_deg_lateral", 0.);
    declare_parameter("end_deg_lateral", 0.);
    declare_parameter("stop_ray_th", 0.0);
    declare_parameter("wheel_separation", 0.0);
    declare_parameter("distance_to_skip", 0.0);
    declare_parameter("open_place_distance", 0.0);
    declare_parameter("select_angvel", std::vector<double>(2, 0.0));
    declare_parameter("detection_div_deg", std::vector<double>(2, 0.0));
    declare_parameter("front_search_angle_start", -15.);
    declare_parameter("front_search_angle_end", 15.);

    // Parameters for Controller
    declare_parameter("wt_controller.linear_vel.max", 0.2);
    declare_parameter("wt_controller.angular_vel.max", 1.0);
    declare_parameter("wt_controller.kp", 10.);
    declare_parameter("wt_controller.search_angle.front.start", -20.);
    declare_parameter("wt_controller.search_angle.front.end", 20.);
    declare_parameter("wt_controller.search_angle.side.start", 69.);
    declare_parameter("wt_controller.search_angle.side.end", 80.);
    declare_parameter("wt_controller.obstacle_avoidance_threshold", 0.6);
    declare_parameter("wt_controller.target_wall_distance", 0.5);

    // Parameters for PotentialController
    declare_parameter("potential.critical_distance", 30.0);
    declare_parameter("potential.repulsive_gain", 1.0);
    declare_parameter("potential.attractive_gain", 3.0);
    declare_parameter("potential.repulsive_min_distance", 0.25);
    declare_parameter("potential.sigma", 0.3);

    // Parameters for Open Place Checker
    declare_parameter("open_place_checker.detect_angle_start", -45.0);
    declare_parameter("open_place_checker.detect_angle_end", 45.0);
    declare_parameter("open_place_checker.detect_angle_division_num", 5);
    declare_parameter("open_place_checker.range_th", 15.0);
    declare_parameter("open_place_checker.ratio_th", 0.7);
}

void WallTracking::get_param()
{
    // this->get_parameter("max_linear_vel", max_linear_vel_);
    // this->get_parameter("max_angular_vel", max_angular_vel_);
    // this->get_parameter("min_angular_vel", min_angular_vel_); // will delete
    // this->get_parameter("distance_from_wall", distance_from_wall_);
    // this->get_parameter("distance_to_stop", distance_to_stop_);
    // this->get_parameter("sampling_rate", sampling_rate_); // will delete
    // this->get_parameter("kp", kp_);
    // this->get_parameter("ki", ki_); // will delete
    // this->get_parameter("kd", kd_); // will delete
    // this->get_parameter("start_deg_lateral", start_deg_lateral_);
    // this->get_parameter("end_deg_lateral", end_deg_lateral_);
    // this->get_parameter("stop_ray_th", stop_ray_th_);
    // this->get_parameter("wheel_separation", wheel_separation_);
    // this->get_parameter("distance_to_skip", distance_to_skip_);
    // this->get_parameter("open_place_distance", open_place_distance_);
    // this->get_parameter("detection_div_deg", detection_div_deg_);
    // this->get_parameter("select_angvel", select_angvel_);

    // For Controller
    float lin_max_vel = get_parameter("wt_controller.linear_vel.max").as_double();
    float ang_max_vel = get_parameter("wt_controller.angular_vel.max").as_double();
    float kp = get_parameter("wt_controller.kp").as_double();
    float front_search_angle_start = get_parameter("wt_controller.search_angle.front.start").as_double() * M_PI / 180.;
    float front_search_angle_end = get_parameter("wt_controller.search_angle.front.end").as_double() * M_PI / 180.;
    float side_search_angle_start = get_parameter("wt_controller.search_angle.side.start").as_double() * M_PI / 180.;
    float side_search_angle_end = get_parameter("wt_controller.search_angle.side.end").as_double() * M_PI / 180.;
    float obs_avoid_th = get_parameter("wt_controller.obstacle_avoidance_threshold").as_double();
    float target_wall_dist = get_parameter("wt_controller.target_wall_distance").as_double();
    controller_.reset(new Controller(
        lin_max_vel, 
        ang_max_vel, 
        kp, 
        front_search_angle_start, 
        front_search_angle_end, 
        side_search_angle_start, 
        side_search_angle_end, 
        obs_avoid_th, 
        target_wall_dist));

    // For Potential Controller
    float critical_distance = get_parameter("potential.critical_distance").as_double(); 
    float repulsive_gain = get_parameter("potential.repulsive_gain").as_double();
    float attractive_gain = get_parameter("potential.attractive_gain").as_double();
    float min_distance = get_parameter("potential.repulsive_min_distance").as_double();
    float sigma = get_parameter("potential.sigma").as_double();
    potential_controller_.reset(new PotentialController(
        critical_distance, repulsive_gain, attractive_gain, 
        min_distance, lin_max_vel, ang_max_vel, sigma));

    // For Open Place Chekcker
    float detect_angle_start = get_parameter("open_place_checker.detect_angle_start").as_double() * M_PI / 180.0;
    float detect_angle_end = get_parameter("open_place_checker.detect_angle_end").as_double() * M_PI / 180.0;
    int detect_angle_division_num = get_parameter("open_place_checker.detect_angle_division_num").as_int();
    float range_th = get_parameter("open_place_checker.range_th").as_double();
    float ratio_th = get_parameter("open_place_checker.ratio_th").as_double();
    open_place_checker_.reset(new OpenPlaceChecker(
        detect_angle_start, 
        detect_angle_end, 
        detect_angle_division_num, 
        range_th, 
        ratio_th
    ));
}

void WallTracking::init_sub()
{
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::QoS(10),
        std::bind(&WallTracking::scan_callback, this, std::placeholders::_1));
    gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "gnss/fix", rclcpp::QoS(10),
        std::bind(&WallTracking::gnss_callback, this, std::placeholders::_1));
    gnss_pose_with_covariance_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "gnss_pose_with_covariance", rclcpp::QoS(10),
        std::bind(&WallTracking::gnss_pose_with_covariance_callback, this, std::placeholders::_1));
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", rclcpp::QoS(1),
        std::bind(&WallTracking::goal_pose_callback, this, std::placeholders::_1));
}

void WallTracking::init_pub()
{
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
    open_place_arrived_pub_ = this->create_publisher<std_msgs::msg::Bool>("open_place_arrived", rclcpp::QoS(10));
    open_place_detection_pub_ = this->create_publisher<std_msgs::msg::String>("open_place_detection", rclcpp::QoS(10));
    behavior_stamped_array_pub_ = this->create_publisher<wall_tracking_msgs::msg::BehaviorStamped>("behavior_stamped", rclcpp::QoS(10));
}

void WallTracking::init_action()
{
    wall_tracking_action_srv_ = rclcpp_action::create_server<WallTrackingAction>(
        this, "wall_tracking",
        std::bind(&WallTracking::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&WallTracking::handle_cancel, this, std::placeholders::_1),
        std::bind(&WallTracking::handle_accepted, this, std::placeholders::_1));
    navigation_action_client_ = rclcpp_action::create_client<NavigateToPose>(
        this,
        "navigate_to_pose");
    nav_send_goal_options_ = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    using namespace std::placeholders;
    nav_send_goal_options_.goal_response_callback = std::bind(
        &WallTracking::goalResponceCallback, this, _1);
    nav_send_goal_options_.result_callback = std::bind(
        &WallTracking::resultCallback, this, std::placeholders::_1);
    nav_send_goal_options_.feedback_callback = std::bind(
        &WallTracking::feedbackCallback, this, 
        std::placeholders::_1, std::placeholders::_2);
}

void WallTracking::behaviorStampedPub(void)
{
    rclcpp::Rate pub_rate(1);
    for(auto &b: behavior_stamped_array_){
        wall_tracking_msgs::msg::BehaviorStamped tmp;
        tmp.behavior_name = b.behavior_name;
        tmp.stamp = b.stamp;
        behavior_stamped_array_pub_->publish(tmp);
        pub_rate.sleep();
    }
    behavior_stamped_array_.clear();
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
    gnss_nan_ = true;
}

void WallTracking::pub_cmd_vel(float linear_x, float angular_z)
{
    geometry_msgs::msg::Twist cmd_vel;
    // linear_x, angular_z: クランプ済み
    cmd_vel.linear.x = linear_x;
    cmd_vel.angular.z = angular_z;
    // RCLCPP_WARN(get_logger(), "linear_x: %f, angular_z: %f", linear_x, angular_z);
    // RCLCPP_WARN(get_logger(), "linear_x: %f, angular_z: %f", cmd_vel.linear.x, cmd_vel.angular.z);
    cmd_vel_pub_->publish(cmd_vel);
}

void WallTracking::scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg) 
{
    if (!init_scan_data_) {
        scan_data_.reset(new ScanData());
        init_scan_data_ = true;
        RCLCPP_INFO(this->get_logger(), "initialized scan data");
    }
    scan_data_->dataUpdate(msg);
}

void WallTracking::gnss_callback(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
    outdoor_ = msg->position_covariance_type == msg->COVARIANCE_TYPE_UNKNOWN ? false : true;
    // RCLCPP_INFO(this->get_logger(), "outdoor: %d", outdoor_);
}

void WallTracking::goal_pose_callback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
    nav_goal_msgs_.pose = *msg;
    behavior_stamped_array_.clear();
    addBehaviorStamedArray("Navigation Start");
    recieved_nav_goal_ = true;
    RCLCPP_INFO(this->get_logger(), "Recieved nav goal: %d", recieved_nav_goal_);
	if(!wall_tracking_flg_) navigation_action_client_->async_send_goal(nav_goal_msgs_, nav_send_goal_options_);
}

void  WallTracking::gnss_pose_with_covariance_callback(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
    if(std::isnan(msg->pose.pose.position.x) && std::isnan(msg->pose.pose.position.y)) gnss_nan_ = true;
    else gnss_nan_ = false;
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
    rclcpp::sleep_for(100ms);
}

void WallTracking::wallTracking()
{
    // RCLCPP_INFO(this->get_logger(), "wall tracking");
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
    if(!init_scan_data_){
        RCLCPP_INFO(this->get_logger(), "Waiting Initial Scan Message ...");
        return;
    }
    std::string detection_res = "Indoor";
    std::shared_ptr<ScanData> tmp_scan = scan_data_;
    std::array<float, 2> cmd_vel;
    controller_->set_scan(tmp_scan);
    open_place_ = false;
    if(!outdoor_){
        cmd_vel = controller_->get_cmd_vel();
    }else{
        open_place_checker_->set_scan(tmp_scan);
        if(open_place_checker_->is_open_place_available()){
            // If available, calculate cmd_vel by PotentialController
            RCLCPP_INFO(get_logger(), "Open place detected. Calculate cmd_vel.");
            open_place_ = true;
            std::array<float, 3> open_laser_info = open_place_checker_->get_open_laser_info();
            // potential_controller_->set_scan(debug_ ? tmp_scan : tmp_integ_scan);
            potential_controller_->set_scan(tmp_scan);
            cmd_vel = potential_controller_->get_cmd_vel(open_laser_info);
            detection_res = "Detect open place";
        }else{
            // If not available, wall tracking
            RCLCPP_WARN(get_logger(), "No open place detected. Wall following.");
            // controller_->set_scan(debug_ ? tmp_scan : tmp_integ_scan);
            cmd_vel = controller_->get_cmd_vel();
        }
    }
    // RCLCPP_WARN(get_logger(), "linear_x: %f, angular_z: %f", cmd_vel[0], cmd_vel[1]);
    pub_cmd_vel(cmd_vel[0], cmd_vel[1]);
    pub_open_place_detection(detection_res);
}

void WallTracking::addBehaviorStamedArray(std::string behavior_name)
{
    wall_tracking_msgs::msg::BehaviorStamped tmp_behavior_stamped;
    tmp_behavior_stamped.behavior_name = behavior_name;
    tmp_behavior_stamped.stamp = now();
    behavior_stamped_array_.push_back(tmp_behavior_stamped);
    // RCLCPP_INFO(this->get_logger(), "Num of behavior stamped array: %ld", behavior_stamped_array_.size());
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

void WallTracking::execute(
    const std::shared_ptr<GoalHandleWallTracking> goal_handle)
{
    const auto debug = goal_handle->get_goal()->start;
    if(!debug){
        while(!recieved_nav_goal_){
            RCLCPP_INFO(this->get_logger(), "Waiting for Navigation Gaol...");
        }
    }
    if(!debug && recieved_nav_goal_){
        navigation_action_client_->async_cancel_all_goals();
        RCLCPP_INFO(this->get_logger(), "Send cancel navigation to server");
    }
    rclcpp::sleep_for(1000ms);
    addBehaviorStamedArray("WallTracking Start");
    RCLCPP_INFO(this->get_logger(), "EXECUTE");
    const auto goal = goal_handle->get_goal();
    std::shared_ptr<WallTrackingAction::Feedback> feedback = std::make_shared<WallTrackingAction::Feedback>();
    auto result = std::make_shared<WallTrackingAction::Result>();
    wall_tracking_flg_ = true;
    rclcpp::Rate loop_rate(20);
    while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
            wall_tracking_flg_ = false;
            result->get = false;
            goal_handle->canceled(result);
            pub_cmd_vel(0.0, 0.0);
            addBehaviorStamedArray("WallTracking Cancel");
            RCLCPP_INFO(this->get_logger(), "Goal Canceled");
            if(!debug && recieved_nav_goal_){
                navigation_action_client_->async_send_goal(nav_goal_msgs_, nav_send_goal_options_);
                addBehaviorStamedArray("Navigation Resume");
                RCLCPP_INFO(this->get_logger(), "Resume navigation");
            }
            return;
        }
        feedback->open_place_arrived = open_place_;
        navigateOpenPlace();
        goal_handle->publish_feedback(feedback);
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
