# SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package = "wall_tracking"
    config = os.path.join(
        get_package_share_directory(package), 
        "config", 
        "wall_tracking.param.yaml"
    )
    
    node = Node(
        package=package, 
        name="wall_tracking", 
        executable="wall_tracking_exe", 
        parameters=[config]
    )
    
    ld = LaunchDescription()
    ld.add_action(node)
    
    return ld