#include"turtlebot3_wall_tracking/wall_tracking.hpp"

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Turtlebot3WallTracking::WallTracking>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}