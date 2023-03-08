#include"turtlebot3_wall/run_along_wall.hpp"
#include<iostream>

namespace Turtlebot3Wall
{
    void RunAlongWall::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        data0 = msg->ranges[0];
        data90 = msg->ranges[90];

        cmd_vel_msg.linear.x = (data0 < distance_to_stop) ? 0.0 : max_linear_vel;

        double sum = 0;
        for (size_t i = 0; i < 20; i++)
        {
            if (msg->ranges[35+i] != INFINITY && msg->ranges[35+i] != NAN)
                sum += msg->ranges[35+i] * sin((35+i)*(M_PI/180));  
            else
                sum += 3.5;  
        }

        RCLCPP_INFO(get_logger(), "range: %f", sum/40);
        
        e = sum/40 - distance_from_wall;
        ei += e * sampling_rate;
        ed = e / sampling_rate;

        float ang_z = e*kp + ei*ki + ed*kd;
        // RCLCPP_INFO(get_logger(), "angular velocity: %f", ang_z);
        if(ang_z > max_angular_vel) ang_z = max_angular_vel;
        if(ang_z < min_angular_vel) ang_z = min_angular_vel;
        cmd_vel_msg.angular.z = ang_z;

        cmd_vel_pub_->publish(cmd_vel_msg);
    }

    RunAlongWall::RunAlongWall()
    : Node("run_along_wall"), count_(0)
    {
        set_param();
        get_param();
        init_scan_sub();
        init_cmd_vel_pub();
        init_variable();
    }

    void RunAlongWall::set_param()
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
    }

    void RunAlongWall::get_param()
    {
        max_linear_vel = get_parameter("max_linear_vel").as_double();
        max_angular_vel = get_parameter("max_angular_vel").as_double();
        min_angular_vel = get_parameter("min_angular_vel").as_double();
        distance_from_wall = get_parameter("distance_from_wall").as_double();
        distance_to_stop = get_parameter("distance_to_stop").as_double();
        sampling_rate = get_parameter("sampling_rate").as_double();
        kp = get_parameter("kp").as_double();
        ki = get_parameter("ki").as_double();
        kd = get_parameter("kd").as_double();
    }

    void RunAlongWall::init_scan_sub()
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(10), std::bind(&RunAlongWall::scan_callback, this, std::placeholders::_1));
    }

    void RunAlongWall::init_cmd_vel_pub()
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtlebot3/cmd_vel", rclcpp::QoS(10));
    }

    void RunAlongWall::init_variable()
    {
        e = 0.0;
        ei = 0.0;
        ed = 0.0;
        ang_z = 0.0;
        pre_ang_z = 0.0;
    }
}
