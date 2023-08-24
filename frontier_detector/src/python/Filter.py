#!/usr/bin/env python3

# jplaced
# 2022, Universidad de Zaragoza

# The filter nodes receives the detected frontier points from all the detectors,
# filters the points, and passes them to the assigner node to command the robots.
# Filtration includes the deletion of old and invalid points, and it also
# discards redundant points.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import rclpy
import tf2_ros
import sys
from tf2_ros import Buffer
# import dynamic_reconfigure.client

import numpy as np
from sklearn.cluster import MeanShift
from copy import copy, deepcopy
from scipy.spatial.transform import Rotation

from Functions import gridValue, createMarker, informationGain, informationGain_NUMBA, yawBtw2Points

from SimpleRobot import Robot

from orb_slam2_ros.msg import PointArray
from visualization_msgs.msg import Marker

from nav_msgs.msg import OccupancyGrid
from tf2_geometry_msgs import Point, PointStamped, Pose
# from dynamic_reconfigure.server import Server

# from frontier_detector.cfg import informationGainConfig

from functools import partial
import time

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Callbacks~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
map_data_ = OccupancyGrid()
frontiers_ = []
f_timestamps_ = []
global_map_ = []
max_t_ = 0

INFORMATION_THRESHOLD_ = 0.5 #0.5


# def reconfigureCallback(config, level=0):
#     global INFORMATION_THRESHOLD_
#     node.get_logger().warn_throttle(0.5, node.get_name() + """: Reconfigure Request! InfoGain threshold changed to: {ig_threshold}""".format(**config))
#     INFORMATION_THRESHOLD_ = config["ig_threshold"]
#     return config


def frontiersCallBack(data:PointStamped, args):
    node = args[2]

    global frontiers_, f_timestamps_

    # transformed_point = args[0].transformPoint(args[1], data)
    transformed_point = args[0].transform(data, args[1])

    x = np.array([transformed_point.point.x, transformed_point.point.y])
    x_t = data.header.stamp._sec

    # Only add if not already there
    # Use temp variables to avoid errors due to global variables dimension (multiple ROS cb same time)
    temp_black = copy(x.tolist())
    temp_array = np.asarray(copy(frontiers_)).tolist()
    temp_time = copy(f_timestamps_)

    if len(temp_array) == len(temp_time):
        assert (len(temp_array) == len(temp_time))
        if temp_black in temp_array:
            repeated_idx = temp_array.index(temp_black)
            temp_time[repeated_idx] = x_t
        else:  # Otherwise, update timestamp
            temp_array.append(x)
            temp_time.append(x_t)

        # Delete too old points
        original_len = len(temp_array)
        assert original_len == len(temp_time)
        for ip in range(0, original_len):
            i = ip - original_len + len(temp_array)
            t_diff = np.abs(temp_time[i] - node.get_clock().now().to_msg().sec)
            if t_diff > max_t_:
                node.get_logger().debug(node.get_name() + ': Deleted a frontier with timestamp diff = ' + str(t_diff))
                del temp_array[i]
                del temp_time[i]

        frontiers_ = copy(temp_array)
        f_timestamps_ = copy(temp_time)
        assert (len(frontiers_) == len(f_timestamps_))

    else:
        node.get_logger().error(node.get_name() + ': Frontier callback failed due to dimension mismatch of ' + str(
            len(temp_array) - len(temp_time)) + '. Skipping callback.')


def mapCallBack(data):
    global map_data_
    map_data_ = data



def globalMapCallback(data):
    global global_map_
    global_map_ = data


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Node~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def node(args=None):
    global frontiers_, map_data_, global_map_, max_t_
    # rclpy.init_node('filter')
    rclpy.init(args=args)
    node = rclpy.create_node('filter')

    # node.declare_parameter('ig_threshold', 0.5)
    node.declare_parameter('ig_threshold', rclpy.Parameter.Type.DOUBLE)

    # Fetch all parameters
    map_topic = node.declare_parameter('~map_topic', '/map').value
    threshold = node.declare_parameter('~costmap_clearing_threshold', 70).value
    # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good,
    # info gain won't be accurate
    info_radius = node.declare_parameter('~info_radius', 1.0).value
    ig_th = node.declare_parameter('~information_threshold', 1.0).value
    goals_topic = node.declare_parameter('~goals_topic', '/detected_points').value
    n_robots = node.declare_parameter('~n_robots', 1).value
    namespace = node.declare_parameter('~namespace', '/robot_').value
    rate_hz = node.declare_parameter('~rate', 1).value
    robot_frame = node.declare_parameter('~robot_frame', 'base_link').value
    global_costmap_topic = node.declare_parameter('~global_costmap_topic', '/global_costmap/costmap').value
    use_gpu = node.declare_parameter('~enable_gpu_comp', True).value
    max_t_ = node.declare_parameter('~maximum_time_f', 1.0).value

    # srv = Server(informationGainConfig, reconfigureCallback)

    rate = node.create_rate(rate_hz)
    # node.create_subscription(OccupancyGrid, "/GridMapper_node/rectified_map", mapCallBack , 10)
    node.create_subscription(OccupancyGrid, "map", mapCallBack , 10)

    global_map_ = OccupancyGrid()

    node.create_subscription(OccupancyGrid, global_costmap_topic, globalMapCallback ,  10)

    # Robot
    robot_name = namespace + str(n_robots)
    robot = Robot(robot_name,node)
    # rclpy.spin(robot)
    # robot.destroy_node()
    # wait_rate = node.create_rate(0.5)

    # Wait if map map has not been received yet
    while len(map_data_.data) < 1:
        rclpy.spin_once(node)
        node.get_logger().info(node.get_name() + ': Filter is waiting for the map.')
        # rate.sleep()
        pass

    # Wait if global costmap map has not been received yet
    while len(global_map_.data) < 1:
        rclpy.spin_once(node)
        node.get_logger().info(node.get_name() + ': Filter is waiting for the global costmap.')
        # rate.sleep()
        pass

    node.get_logger().info(node.get_name() + ': Filter received local and global costmaps.')

    global_frame = "/" + map_data_.header.frame_id
    tfBuffer = Buffer()
    tf_lstnr = tf2_ros.TransformListener(tfBuffer,node)
    # tf_lstnr.waitForTransform(global_frame[1:], robot_frame, rclpy.Time(0), rclpy.Duration(10))
    tfBuffer.wait_for_transform_async(global_frame[1:], robot_frame, rclpy.time.Time())

    # node.create_subscription(PointStamped, goals_topic,  callback=frontiersCallBack, callback_args=[tf_lstnr, global_frame[1:]])
    node.create_subscription(PointStamped, goals_topic,lambda data: frontiersCallBack(data,[tfBuffer, global_frame[1:],node] ), 1 )

    # Publishers
    pub_frontiers = node.create_publisher(Marker, node.get_name() + '/frontiers',  1)
    pub_centroids = node.create_publisher(Marker,node.get_name() + '/centroids',  1)
    pub_filt_points = node.create_publisher(PointArray,node.get_name() + '/filtered_points',  1)

    # Wait if no frontier is received yet
    counter = 0
    while len(frontiers_) < 1:
        rclpy.spin_once(node)
        if counter == 0:
            node.get_logger().info(node.get_name() + ': Filter is waiting for frontiers.')
            counter = 1

    node.get_logger().info(node.get_name() + ': Filter received frontiers.')

    points = createMarker(frame=map_data_.header.frame_id, ns="raw_frontiers", colors=[1.0, 1.0, 0.0], scale=0.2,
                          lifetime=1 / rate_hz, node=node)
    points_clust = createMarker(frame=map_data_.header.frame_id, ns="filtered_frontiers", colors=[0.0, 1.0, 0.0],
                                lifetime=1 / rate_hz, node=node)

    p = Point()
    p.z = float(0)

    temp_point_stamped = PointStamped()
    temp_point_stamped.header.frame_id = map_data_.header.frame_id
    temp_point_stamped.header.stamp = node.get_clock().now().to_msg()
    temp_point_stamped.point.z = 0.0

    temp_point_array = PointArray()
    temp_point = Point()
    temp_point.z = 0.0

    # client = dynamic_reconfigure.client.Client("filter", timeout=1, config_callback=reconfigureCallback)
    # client.update_configuration({"ig_threshold": ig_th})

    my_new_param = rclpy.parameter.Parameter(
            'ig_threshold',
            rclpy.Parameter.Type.DOUBLE,
            ig_th
        )
    all_new_parameters = [my_new_param]
    node.set_parameters(all_new_parameters)

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    while rclpy.ok():
        rclpy.spin_once(node)
        # Clustering frontier points
        centroids = []
        front = deepcopy(frontiers_)

        if len(front) > 1:
            ms = MeanShift(bandwidth=1.5)
            ms.fit(front)
            centroids = ms.cluster_centers_
        elif len(front) == 1:  # If there is only one frontier no need for clustering
            centroids = front

        # frontiers_ = deepcopy(centroids)

        # Clearing frontiers
        original_centroids_len = len(centroids)
        for zp in range(0, original_centroids_len):
            
            z = zp - original_centroids_len + len(centroids)

            cond = False
            temp_point_stamped.point.x = centroids[z][0]
            temp_point_stamped.point.y = centroids[z][1]

            # Remove frontiers at ~(0,0) // probably errors of the RRT frontier detectors
            if abs(centroids[z][0]) + abs(centroids[z][1]) < .1:
                cond = True

            # Remove frontiers too close to the robot
            if not cond:
                distance = np.linalg.norm(robot.getPosition() - centroids[z])
                if distance < 0.25:
                    cond = True
                    # node.get_logger().warn(node.get_name() + ": Will delete a centroid too close to the robot.")

            # Remove any frontier inside an occupied cell (threshold)
            if not cond:
                # transformed_point = tf_lstnr.transformPoint(global_map_.header.frame_id, temp_point_stamped)
                transformed_point = tfBuffer.transform(temp_point_stamped, global_map_.header.frame_id)
                x = np.array([transformed_point.point.x, transformed_point.point.y])
                gval = gridValue(global_map_, x)
                if gval >= threshold:
                    cond = True
                    # node.get_logger().warn(node.get_name() + ': Will delete a frontier in occupied cell with value: '
                                #   + str(gval))

            # Remove frontiers with low information gain
            if not cond:
                # if use_gpu:
                if False:
                    ig = informationGain_NUMBA(map_data_.info.resolution, map_data_.info.width,
                                               map_data_.info.origin.position.x,
                                               map_data_.info.origin.position.y, np.array(map_data_.data),
                                               centroids[z][0], centroids[z][1], 1.0)
                else:
                    ig = informationGain(map_data_, [centroids[z][0], centroids[z][1]], 1.0)
                if ig < INFORMATION_THRESHOLD_:
                    cond = True
                    # node.get_logger().warn(node.get_name() + ': Will delete a frontier with information gain = ' + str(ig))

            # Remove frontiers in unreachable locations
            if not cond:
            # if False:
                p_frontier = np.array([centroids[z][0], centroids[z][1]])
                pose_frontier = Pose()
                pose_frontier.position.x = p_frontier[0]
                pose_frontier.position.y = p_frontier[1]
                pose_frontier.position.z = float(0)
                # Add orientation at frontier's location
                R_frontier = Rotation.from_euler('xyz', [0., 0., yawBtw2Points(robot.getPosition(), p_frontier)],
                                                 degrees=False)
                q_frontier = R_frontier.as_quat()
                pose_frontier.orientation.x = q_frontier[0]
                pose_frontier.orientation.y = q_frontier[1]
                pose_frontier.orientation.z = q_frontier[2]
                pose_frontier.orientation.w = q_frontier[3]
                plan = robot.makePlan(robot.getPoseAsGeometryMsg(), pose_frontier)
                if plan == None:
                    cond = True
                    # node.get_logger().warn(node.get_name() + ': Will delete an unreachable frontier.')
                else:
                    n_points = int(len(plan))
                    # node.get_logger().info(str(n_points))
                    if n_points == 0:
                        cond = True
                        # node.get_logger().warn(node.get_name() + ': Will delete an unreachable frontier.')

            if cond:
                centroids = np.delete(centroids, z, axis=0)
                # node.get_logger().warn(node.get_name() + ': Frontier deleted.')

        node.get_logger().debug(node.get_name() + ': Frontier centroids len=' + str(len(centroids)) + ', frontiers len=' + str(
            len(front)) + '. Centroids: \n' + str(centroids))

        # Publishing
        temp_point_array.points = []
        pp = []
        if len(centroids) > 0:
            for i in centroids:
                temp_point.x = i[0]
                temp_point.y = i[1]
                temp_point_array.points.append(copy(temp_point))
                pp.append(copy(temp_point))

        points_clust.id += 1
        points_clust.points = pp
        pub_centroids.publish(points_clust)
        pub_filt_points.publish(temp_point_array)

        pp = []
        if len(front) > 0:
            for q in front:
                p.x = q[0]
                p.y = q[1]
                pp.append(copy(p))

        points.id += 1
        points.points = pp
        pub_frontiers.publish(points)
        time.sleep(1)
        # rate.sleep()


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Main~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if __name__ == '__main__':
    try:
        node()
    except rclpy.exceptions.ROSInterruptException:
        pass
