#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter

from ament_index_python.packages import get_package_share_directory

from launch.actions import TimerAction

import os


def generate_launch_description():

    config_dir = os.path.join(get_package_share_directory('rosbot_gazebo'),'params')
    params_file = os.path.join(config_dir,'laser.yaml')
    # params_file = os.path.join(config_dir,'edit_3.yaml')
    # params_file = os.path.join(config_dir,'edit_2.yaml')



    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("nav2_bringup"),
                    "launch",
                    "navigation_launch.py",
                ]
            )
        ),
        launch_arguments={
            # "namespace": "",
            # "use_sim_time": "true",
            # "autostart": "true",
            "params_file": params_file,
            # "use_lifecycle_mgr": "false",
            # "map_subscribe_transient_local": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            nav2
        ]
    )
