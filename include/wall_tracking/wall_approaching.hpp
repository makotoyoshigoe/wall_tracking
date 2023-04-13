#include<rclcpp/rclcpp.hpp>

namespace WallTracking
{
class WallApproaching : public rclcpp::Node
{
    public:
        WallApproaching();

    protected:
        void handle_goal();
    
    private:
        size_t count_;
};
}