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
        default_value="/GridMapper_node/rectified_map",
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
    ##################################
    robot_frame = LaunchConfiguration("robot_frame")
    declare_robot_frame_arg = DeclareLaunchArgument(
        "robot_frame",
        default_value="base_link",
    )
    local_eta = LaunchConfiguration("local_eta")
    declare_local_eta_arg = DeclareLaunchArgument(
        "local_eta",
        default_value="0.75",
    )
    global_eta = LaunchConfiguration("global_eta")
    declare_global_eta_arg = DeclareLaunchArgument(
        "global_eta",
        default_value="1.0",
    )
    rate_detectors_hz = LaunchConfiguration("rate_detectors_hz")
    declare_rate_detectors_hz_arg = DeclareLaunchArgument(
        "rate_detectors_hz",
        default_value="10",
    )
    rate_filter_hz = LaunchConfiguration("rate_filter_hz")
    declare_rate_filter_hz_arg = DeclareLaunchArgument(
        "rate_filter_hz",
        default_value="10",
    )
    rate_octo_hz = LaunchConfiguration("rate_octo_hz")
    declare_rate_octo_hz_arg = DeclareLaunchArgument(
        "rate_octo_hz",
        default_value="1",
    )
    rate_grid_hz = LaunchConfiguration("rate_grid_hz")
    declare_rate_grid_hz_arg = DeclareLaunchArgument(
        "rate_grid_hz",
        default_value="2",
    )

    octomapper = Node(
        package="frontier_detector",
        executable="frontier_detector_octomapper",
        # remappings=[('/map_points', '/RGBD/map_points'),
        #             ("/info/state", "/RGBD/info/state"),
        #             ("/gba_running", "/RGBD/gba_running")
        #             ],
        parameters=[{"/rate": rate_octo_hz},
                    {"/target_frame_id": robot_frame},
                    {"/pointcloud_frame_id": global_frame},
                    {"/octomap/rebuild": "false"},
                    {"/max_height": "3.0"}],
        output="screen",
    )

    gridmapper = Node(
        package="frontier_detector",
        executable="frontier_detector_gridmapper",
        # remappings=[("/gba_running", "/orb_slam2_"+camera_type+"/gba_running")],
        parameters=[{"/rate": rate_octo_hz},
                    {"/occupancy/publish_occupancy_grid": "true"},
                    {"/occupancy/publish_occupancy_grid_rect": "true"},
                    {"/target_frame_id": robot_frame},
                    {"/pointcloud_frame_id": global_frame},
                    {"/occupancy/projected_map/min_height": "0.12"},
                    {"/occupancy/projected_map/max_height": "2.1"}],
        output="screen",
    )

    ns = LaunchConfiguration("ns")
    declare_ns_arg = DeclareLaunchArgument(
        "ns",
        # default_value="frontier_detectors",
        default_value="",
    )

    OpenCV_detector = Node(
            package='frontier_detector',
            executable='OpenCV_detector.py',
            name='opencv_detector',
            output="screen",
            parameters=[{"map_topic": map_topic_name},
                        {"rate": rate_detectors_hz}],
            namespace=ns,
    )
    
    global_detector = Node(
            package='frontier_detector',
            executable='frontier_detector_global_rrt',
            name='global_detector',
            output="screen",
            parameters=[{"map_topic": map_topic_name},
                        {"rate": rate_detectors_hz},
                        {"eta": global_eta}],
            namespace=ns,
    )
    
    local_detector = Node(
            package='frontier_detector',
            executable='frontier_detector_local_rrt',
            name='local_detector',
            output="screen",
            parameters=[{"map_topic": map_topic_name},
                        {"robot_frame": robot_frame},
                        {"rate": rate_detectors_hz},
                        {"eta": global_eta}],
            namespace=ns,
    )
    
    filter = Node(
            package='frontier_detector',
            executable='Filter.py',
            name='filter',
            output="screen",
            respawn_delay="3",
            parameters=[{"global_costmap_topic": "/global_costmap/costmap"},
                        {"costmap_clearing_threshold": "70"},
                        {"information_threshold": "0.5"},
                        {"enable_gpu_comp": gpu},
                        {"map_topic": map_topic_name},
                        {"goals_topic": "detected_points"},
                        {"rate": rate_filter_hz},
                        {"n_robots": robot_number},
                        {"info_radius": "0.5"},
                        {"max_t": "1.0"}],
            namespace=ns,
    )
    
    initializer = Node(
            package='frontier_detector',
            executable='Initializer.py',
            name='point_init',
            output="screen",
            # parameters=[{"rate": "1.0"}],
            namespace=ns,
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
            declare_robot_frame_arg,
            declare_local_eta_arg,
            declare_global_eta_arg,
            declare_rate_detectors_hz_arg,
            declare_rate_filter_hz_arg,
            declare_rate_octo_hz_arg,
            declare_rate_grid_hz_arg,
            octomapper,
            gridmapper,
            declare_ns_arg,
            OpenCV_detector,
            global_detector,
            local_detector,
            initializer,
            filter,

            # TimerAction(
            #     period=10.0,
            #     actions=[filter]
            # ),

        ]
    )
