#!/usr/bin/env python3

# jplaced
# 2022, Universidad de Zaragoza

# This node receives target exploration goals, which are the filtered frontier
# points published by the filter node, and commands the robots accordingly. The
# controller node commands the robot through the move_base_node.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import rclpy
import tf2_ros
import heapq
import numpy as np
import sys
from rclpy.node import Node
from tf2_ros.buffer import Buffer
# import dynamic_reconfigure.client

from numpy import array
from copy import deepcopy
from scipy.spatial.transform import Rotation

# from frontier_detector.msg import PointArray
from orb_slam2_ros.msg import PointArray
from nav_msgs.msg import OccupancyGrid
"""
from sensor_msgs.msg import PointCloud2
"""
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from visualization_msgs.msg import MarkerArray
from actionlib_msgs.msg import GoalID
from orb_slam2_ros.msg import ORBState

from Functions import waitEnterKey, quaternion2euler, cellInformation_NUMBA, cellInformation, yawBtw2Points
from Robot import Robot
from Map import Map
from WeightedPoseGraph import WeightedPoseGraph

from rclpy.duration import Duration

from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import time

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Callbacks~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
gridMap_data_ = OccupancyGrid()
frontiers_ = []
mapPoints_ = []
vertices_ = []
edges_ = []

is_lost_ = False
is_relocalizing_ = False
rclpy.init(args=None)
node = rclpy.create_node('decision_maker')
goal_cancel_pub_ = node.create_publisher(GoalID, '/robot_1/move_base/cancel', 10)


def mapPointsCallBack(data):
    global mapPoints_

    mapPoints_ = []
    temp = []
    for i in range(0, len(data.data)):
        if data.data[i] == -1.0:
            mapPoints_.append(temp)
            temp = []
        else:
            temp.append(data.data[i])


def vertexCallBack(data):
    global vertices_
    n = data.layout.dim[0].stride

    vertices_ = []
    for i in range(0, len(data.data), n):
        vertices_.append(data.data[i:i + n])


def edgesCallBack(data):
    global edges_
    n = data.layout.dim[0].stride

    edges_ = []
    for i in range(0, len(data.data), n):
        edges_.append(data.data[i:i + n])


def frontiersCallBack(data):
    global frontiers_
    frontiers_ = []
    for point in data.points:
        frontiers_.append(array([point.x, point.y]))


def mapCallBack(data):
    global gridMap_data_
    gridMap_data_ = data


def statusCallBack(data,node:Node):
    """
    UNKNOWN=0, SYSTEM_NOT_READY=1, NO_IMAGES_YET=2, NOT_INITIALIZED=3, OK=4, LOST=5
    """
    goal_cancel_pub_ = node.create_publisher(GoalID, '/robot_1/move_base/cancel', 10)
    global is_lost_, is_relocalizing_
    if data.state == 4:
        is_lost_ = False
        if is_relocalizing_:  # Stop trying to re localize if already OK
            msg = GoalID()
            goal_cancel_pub_.publish(msg)
            is_relocalizing_ = False
            node.get_logger().info(node.get_name() + ': ORB-SLAM re localized successfully.')
    elif data.state == 5 and not is_relocalizing_:  # If lost, cancel current goal and send best re localization goal
        # Empty stamp, empty ID -> cancels ALL goals.
        # https://wiki.ros.org/actionlib/DetailedDescription
        msg = GoalID()
        goal_cancel_pub_.publish(msg)
        is_lost_ = True
        node.get_logger().warn(node.get_name() + ': ORB-SLAM status is LOST. Robot stopped.'
                                                     ' Sending robot to best re localization pose.',throttle_duration_sec = 1)
    elif data.state == 0:  # Stop robot
        msg = GoalID()
        goal_cancel_pub_.publish(msg)
        node.get_logger().warn(node.get_name() + ': ORB-SLAM status is UNKNOWN. Robot stopped.',throttle_duration_sec = 1)


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Node~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def node_run():
    global frontiers_, mapPoints_, vertices_, edges_, gridMap_data_, is_relocalizing_,node
    # rclpy.init_node('decision_maker')

    # rclpy.init(args=sys.argv)
    # node = rclpy.create_node('decision_maker')

    node.get_logger().info(node.get_name() + ": Initializing...")


    # Fetch all parameters
    map_topic = node.declare_parameter('~map_topic', '/map').value
    # map_topic = node.declare_parameter('~map_topic', 'GridMapper_node/rectified_map').value
    frontiers_topic = node.declare_parameter('~frontiers_topic', '/filter/filtered_points').value
    n_robots = node.declare_parameter('~n_robots', 1).value
    namespace = node.declare_parameter('~namespace', 'robot_').value
    rate_hz = node.declare_parameter('~rate', 100).value
    delay_after_assignment = node.declare_parameter('~delay_after_assignment', 0.1).value
    show_debug_path = node.declare_parameter('~show_debug_path', True).value
    exploring_time = node.declare_parameter('~max_exploring_time', 900000).value #9000
    use_gpu = node.declare_parameter('~enable_gpu_comp', False).value
    camera_type = node.declare_parameter('~camera_type', 'RGBD').value
    # camera_type = "rgbd"

    rate = node.create_rate(node.get_parameter('~rate').get_parameter_value().integer_value)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    # delay_rate = node.create_rate(node.get_parameter('~delay_after_assignment').get_parameter_value().integer_value)

    node.create_subscription(OccupancyGrid, map_topic,  mapCallBack ,10)
    node.create_subscription(PointArray, frontiers_topic,  frontiersCallBack,10)
    # node.create_subscription(ORBState,"/orb_slam2_" + camera_type + "/info/state", statusCallBack,10)
    node.create_subscription(ORBState, "/RGBD/info/state", lambda data: statusCallBack(data,node),10)


    # node.create_subscription(Float64MultiArray, "/orb_slam2_" + camera_type + "/vertex_list",  vertexCallBack,10)
    # node.create_subscription(Float64MultiArray, "/orb_slam2_" + camera_type + "/edge_list",  edgesCallBack,10)
    # node.create_subscription(Float64MultiArray, "/orb_slam2_" + camera_type + "/point_list",  mapPointsCallBack,10)

    node.create_subscription(Float64MultiArray, camera_type + "/vertex_list",  vertexCallBack,10)
    node.create_subscription(Float64MultiArray, camera_type + "/edge_list",  edgesCallBack,10)
    node.create_subscription(Float64MultiArray, camera_type + "/point_list",  mapPointsCallBack,10)

    set_cli = node.create_client(SetParameters, '/filter/set_parameters')
    while not set_cli.wait_for_service(timeout_sec=1):
                        node.get_logger().info(f"Waiting for set_cli to become available.")
    # set_cli.wait_for_service()
        
    get_cli = node.create_client(GetParameters, '/filter/get_parameters')
    # get_cli.wait_for_service()
    while not get_cli.wait_for_service(timeout_sec=1):
                    node.get_logger().info(f"Waiting for get_cli to become available.")

    if show_debug_path:
        marker_hallucinated_path_pub_ = node.create_publisher(MarkerArray, 'marker_hallucinated_path', 10)
        """
        point_cloud2_map_pub_ = node.create_publisher("marker_points_frustum", PointCloud2, queue_size=1)
        """
        marker_hallucinated_graph_pub_ = node.create_publisher(MarkerArray, 'marker_hallucinated_graph', 10)

    # Wait if map is not received yet
    while len(gridMap_data_.data) < 1:
        rclpy.spin_once(node)
        pass
    # rclpy.loginfo(node.get_name() + ": Controller received map.")
    node.get_logger().info(node.get_name() + ": Controller received map.")

    # Robot
    robot_name = namespace + str(n_robots)
    robot_ = Robot(robot_name,node)

    # ORB-SLAM map
    map_ = Map(node)

    # Get ROS time in seconds
    # t_0 = rclpy.get_time()
    t_0 = node.get_clock().now()

    ig_changer = 0

    node.get_logger().info(node.get_name() + ": Initialized.")

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    while rclpy.ok():
        rclpy.spin_once(node)
        # Check temporal stopping criterion
        t_f = node.get_clock().now() - t_0  # Get ROS time in seconds
        # if t_f >= Duration(seconds=exploring_time):
        #     robot_.cancelGoal()
        #     waitEnterKey()

        # Get tf from camera link to base frame
        cond = 0
        while cond == 0:
            try:
                transformObject = tf_buffer.lookup_transform("base_link", "camera_link",
                                                                             rclpy.time.Time())
                t_camera_base = transformObject.transform.translation
                q_camera_base = transformObject.transform.rotation
                map_.setCameraBaseTf(t_camera_base, q_camera_base)
                cond = 1
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                node.get_logger().error(node.get_name() + ": Could not get TF from base to camera link.")
                cond = 0

        if not is_lost_:  # ORB-SLAM OK
            centroids = deepcopy(frontiers_)

            n_centroids = len(centroids)
            if n_centroids <= 0:
                # rclpy.logwarn_throttle(0.5, node.get_name() + ": No frontiers.")
                node.get_logger().warn(node.get_name() + ": No frontiers.", throttle_duration_sec = 0.5)
                if ig_changer < 10:
                    ig_changer += 1

                # client = dynamic_reconfigure.client.Client("/frontier_detectors/filter")
                # current_threshold = node.get_parameter('ig_threshold').get_parameter_value().double_value
                # while not get_cli.wait_for_service(timeout_sec=1):
                #     node.get_logger().info(f"Waiting for get_cli to become available.")
                # get_cli.wait_for_service()
                current_threshold = float()
                req = GetParameters.Request()
                req.names = ['ig_threshold']
                res = get_cli.call_async(req)
                rclpy.spin_until_future_complete(node, res)
                current_threshold = res.result()

                # if ig_changer > 10 and client.get_configuration(timeout=1)["ig_threshold"] != 0.1:
                if ig_changer > 10 and current_threshold != 0.1:
                    # client.update_configuration({"ig_threshold": 0.1})
                    # node.set_parameters(rclpy.parameter.Parameter('ig_threshold',rclpy.Parameter.Type.DOUBLE, '0.1'))
                    # while not set_cli.wait_for_service(timeout_sec=1):
                    #     node.get_logger().info(f"Waiting for set_cli to become available.")
                    req = SetParameters.Request()
                    req.parameters = [rclpy.parameter.Parameter('ig_threshold',rclpy.Parameter.Type.DOUBLE, '0.1')]
                    set_cli.call_async(req)
                    rclpy.spin_until_future_complete(node, res)
                    
            else:
                # Get SLAM graph
                map_.setNodes(vertices_)
                map_.setEdges(edges_)
                map_.setMapPoints(mapPoints_)

                nodes, edges = map_.getNodesEdges()

                # Build nx graph
                G = WeightedPoseGraph(nodes, edges, 'd_opt', node)

                # If no nodes (starting step) build graph with one edge at origin.
                n = float(G.getNNodes())
                m = float(G.getNEdges())
                if n < 1:
                    G.graph.add_node(int(0), translation=[0, 0, 0], orientation=[0, 0, 0, 0])

                info_gain = []
                closer_goal = False
                single_goal = False

                # If only one frontier no need to evaluate anything. Select that frontier
                if n_centroids == 1:
                    node.get_logger().warn(node.get_name() + ": Only one frontier detected. Selecting it.")
                    single_goal = True
                # If no edges (starting step) do not evaluate D-opt. Select random frontier
                elif m < 1:
                    node.get_logger().warn(node.get_name() + ": Graph not started yet, m < 1. Selecting goal +=[0.1,0.1].")
                    closer_goal = True
                else:  # Otherwise
                    node.get_logger().info(node.get_name() + ": Computing information gain of every frontier candidate.")
                    for ip in range(0, n_centroids):
                        # Get frontier goal
                        p_frontier = np.array([centroids[ip][0], centroids[ip][1]])

                        # Compute hallucinated pose graph
                        if show_debug_path:
                            if use_gpu:
                                seen_cells_pct = cellInformation_NUMBA(np.array(gridMap_data_.data),
                                                                       gridMap_data_.info.resolution,
                                                                       gridMap_data_.info.width,
                                                                       gridMap_data_.info.origin.position.x,
                                                                       gridMap_data_.info.origin.position.y,
                                                                       p_frontier[0], p_frontier[1], 0.5)
                            else:
                                seen_cells_pct = cellInformation(gridMap_data_, p_frontier, 0.5)

                            hallucinated_path, G_frontier = G.hallucinateGraph(robot_, map_, seen_cells_pct, p_frontier,
                                                                               True)
                            marker_hallucinated_path_pub_.publish(hallucinated_path)
                            marker_hallucinated_graph_pub_.publish(G_frontier.getGraphAsMarkerArray(color=False))
                            # waitEnterKey()
                            """
                            visualizing_pts = map_.getMapPointsAsROSPointCloud2()
                            pts = map_.frustumCulling(robot_.getPoseAsGeometryMsg())
                            visualizing_pts = map_.getMapPointsAsROSPointCloud2("map", pts)
                            point_cloud2_map_pub_.publish(visualizing_pts)
                            waitEnterKey()
                            """
                        else:
                            G_frontier = G.hallucinateGraph(robot_, map_, p_frontier, False)

                        # Compute no. of spanning trees <=> D-opt(FIM)
                        n_frontier = float(G_frontier.getNNodes())
                        if n_frontier > 0:
                            L_anchored = G_frontier.computeAnchoredL()
                            _, t = np.linalg.slogdet(L_anchored.todense())
                            n_spanning_trees = n_frontier ** (1 / n_frontier) * np.exp(t / n_frontier)
                            info_gain.append(n_spanning_trees)

                # Goal sender
                if robot_.getState() == 1:
                    node.get_logger().warn(node.get_name() + ": Robot is not available.")
                elif closer_goal:
                    robot_.sendGoal(robot_.getPosition() + [0.1, 0.1], True)
                elif single_goal:
                    node.get_logger().info(node.get_name() + ": " + format(robot_name) + " assigned to " + format(centroids[0]))
                    robot_.sendGoal(centroids[0], True)
                elif len(info_gain) > 0:
                    # Select next best frontier
                    info_gain_record = []
                    centroid_record = []

                    for ip in range(0, len(centroids)):
                        info_gain_record.append(info_gain[ip])
                        centroid_record.append(centroids[ip])

                    winner_id = info_gain_record.index(np.max(info_gain_record))

                    node.get_logger().info(node.get_name() + ": Information record: " + format(info_gain_record))
                    node.get_logger().info(node.get_name() + ": Centroids record: " + format(centroid_record))
                    node.get_logger().info(node.get_name() + ": " + format(robot_name) + " assigned to " +
                                  format(centroid_record[winner_id]))

                    # Send goal to robot
                    initial_plan_position = robot_.getPosition()
                    robot_.sendGoal(centroid_record[winner_id], True)

                    # If plan fails near to starting position, send new goal to the next best frontier
                    if robot_.getState() != 3:
                        euclidean_d = np.linalg.norm(robot_.getPosition() - initial_plan_position)
                        if euclidean_d <= 2.0:
                            new_goal = 2
                            while robot_.getState() != 3 and new_goal <= len(info_gain_record):
                                second_max = heapq.nlargest(new_goal, info_gain_record)[1]
                                winner_id = info_gain_record.index(second_max)
                                node.get_logger().warn(node.get_name() + ": Goal aborted near previous pose (eucl = " +
                                              str(euclidean_d) + "). Sending new goal to: " +
                                              str(centroid_record[winner_id]))
                                robot_.sendGoal(centroid_record[winner_id], True)
                                new_goal = new_goal + 1

                        else:
                            node.get_logger().warn(node.get_name() + ": Goal aborted away from previous pose (eucl = " +
                                          format(euclidean_d) + "). Recomputing.")

        else:  # ORB-SLAM lost
            is_relocalizing_ = True
            while is_lost_:
                best_reloc_poses = map_.getBestRelocPoses(robot_.getPoseAsGeometryMsg())
                node.get_logger().warn(node.get_name() + ": ORB-SLAM lost. Sending robot to best re localization pose.")
                for reloc_poses in best_reloc_poses:
                    _, _, reloc_yaw = quaternion2euler(reloc_poses.orientation.w, reloc_poses.orientation.x,
                                                       reloc_poses.orientation.y, reloc_poses.orientation.z)
                    reloc_position = [reloc_poses.position.x, reloc_poses.position.y]
                    node.get_logger().info(node.get_name() + ": " + format(robot_name) + " assigned to [" +
                                  format(reloc_position) + ", " + format(reloc_yaw * 180 / 3.14159) + "]")
                    robot_.sendGoalAsPose(reloc_poses, True)
                    time.sleep(delay_after_assignment)
                rclpy.spin_once(node)

        # Wait delay after assignment
        # rclpy.sleep(delay_after_assignment)
        time.sleep(delay_after_assignment)
        rate.sleep()


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Main~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if __name__ == '__main__':
    try:
        node_run()
    except rclpy.exceptions.ROSInterruptException:
        pass
