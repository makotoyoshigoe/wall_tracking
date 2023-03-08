#include"turtlebot3_wall/run_along_wall.hpp"

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Turtlebot3Wall::RunAlongWall>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}