// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/laser_scan.hpp>
#include<geometry_msgs/msg/twist.hpp>
#include<rclcpp/qos.hpp>

namespace WallTracking
{
    class WallTracking : public rclcpp::Node
    {
        public:
            explicit WallTracking();
        
        protected:
            void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
            void set_param();
            void get_param();
            void init_scan_sub();
            void init_cmd_vel_pub();
            void init_variable();

        private:
            size_t count_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
            
            std::string robot_name;
            std::string cmd_vel_topic_name;
            int scan_count;
            float distance_from_wall;
            float distance_to_stop;
            float max_linear_vel;
            float max_angular_vel, min_angular_vel;
            float data90;
            float data0;
            float ang_z, pre_ang_z;
            float sampling_rate;
            float e, ei, ed;
            float kp, ki, kd;
            geometry_msgs::msg::Twist cmd_vel_msg;
    };
}
