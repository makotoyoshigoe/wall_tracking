#!bin/bash 
ros2 action send_goal /wall_tracking wall_tracking_msgs/action/WallTracking {'start: true'}
