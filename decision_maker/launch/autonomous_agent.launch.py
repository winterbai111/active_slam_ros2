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
    namespace_init_count = LaunchConfiguration("namespace_init_count")
    declare_namespace_init_count_arg = DeclareLaunchArgument(
        "namespace_init_count",
        default_value="1",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
    )

    map_topic_name = LaunchConfiguration("map_topic_name")
    declare_map_topic_name_arg = DeclareLaunchArgument(
        "map_topic_name",
        default_value="/gridmapper/rectified_map",
    )

    global_frame = LaunchConfiguration("global_frame")
    declare_global_frame_arg = DeclareLaunchArgument(
        "global_frame",
        default_value="map",
    )

    robot_namespace = LaunchConfiguration("robot_namespace")
    declare_robot_namespace_arg = DeclareLaunchArgument(
        "robot_namespace",
        default_value="robot_1",
    )
    robot_number = LaunchConfiguration("robot_number")
    declare_robot_number_arg = DeclareLaunchArgument(
        "robot_number",
        default_value="1",
    )
    gpu = LaunchConfiguration("gpu")
    declare_gpu_arg = DeclareLaunchArgument(
        "gpu",
        default_value="true",
    )
    camera_type = LaunchConfiguration("camera_type")
    declare_camera_type_arg = DeclareLaunchArgument(
        "camera_type",
        default_value="rgbd",
    )

    camInfo_topic_name = LaunchConfiguration("camInfo_topic_name")
    declare_camInfo_topic_name_arg = DeclareLaunchArgument(
        "camInfo_topic_name",
        default_value="/robot_1/camera/rgb/camera_info",
    )
    ##################################

    frontiers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("frontier_detector"),
                    "launch",
                    "frontiers.launch.py",
                ]
            )
        ),
        # launch_arguments={"map_topic_name": map_topic_name,
        #                   "global_frame":global_frame,
        #                   "robot_namespace":robot_namespace,
        #                   "gpu":gpu,
        #                   "camera_type":camera_type,}.items(),
    )

    G_publisher = Node(
            package='decision_maker',
            executable='GraphDrawer.py',
            name='G_publisher',
            output="screen",
            # parameters=[{"camera_type": camera_type}],
    )

    decision_maker = Node(
            package='decision_maker',
            executable='Controller.py',
            name='decision_maker',
            output="screen",
            parameters=[{"map_topic": map_topic_name},
                        {"cameraInfo_topic": camInfo_topic_name},
                        {"global_frame": global_frame},
                        {"frontiers_topic": "/frontier_detectors/filter/filtered_points"},
                        {"n_robots": robot_number},
                        {"delay_after_assignment": "0.5"},
                        {"rate": "10"},
                        {"type": "random"},
                        {"show_debug_path": "true"},
                        {"exploring_time": "9000"},
                        {"enable_gpu_comp": gpu},
                        {"hallucinated_plan_th": "60"},
                        {"camera_type": camera_type}],
    )

    return LaunchDescription(
        [
            declare_namespace_init_count_arg,
            declare_use_sim_time_arg,
            declare_map_topic_name_arg,
            declare_global_frame_arg,
            declare_robot_namespace_arg,
            declare_robot_number_arg,
            declare_gpu_arg,
            declare_camera_type_arg,
            declare_camInfo_topic_name_arg,
            # frontiers,
            G_publisher,
            decision_maker,

        ]
    )
