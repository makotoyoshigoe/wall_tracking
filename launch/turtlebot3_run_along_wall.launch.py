import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package = "turtlebot3_wall"
    config = os.path.join(
        get_package_share_directory(package), 
        "config", 
        "run_along_wall.param.yaml"
    )
    
    node = Node(
        package=package, 
        name="run_along_wall", 
        executable="run_along_wall", 
        parameters=[config]
    )
    
    ld = LaunchDescription()
    ld.add_action(node)
    
    return ld