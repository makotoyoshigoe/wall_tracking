# SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    package = "nav2_bringup"
    nav2_bringup_launch = os.path.join(get_package_share_directory(package), 'launch')
    map_dir = os.path.join(get_package_share_directory('wall_tracking'), 'map')
    amcl_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch, 'localization_launch.py')
        ), 
        launch_arguments={
            'map': os.path.join(map_dir, 'map_turtlebot3_house.yaml')
        }.items()
    )
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch, 'rviz_launch.py')
        )
    )
    ld = LaunchDescription()
    ld.add_action(amcl_localization_launch)
    # ld.add_action(rviz_launch)
    return ld