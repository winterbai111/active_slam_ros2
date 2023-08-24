#!/usr/bin/env python3

# jplaced
# 2022, Universidad de Zaragoza

# This node draws the pose-graph in RViZ as a marker array. Only for visualization purposes.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import rclpy
import tf2_ros
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float64MultiArray

from Map import Map

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Callbacks~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
vertices_ = []
edges_ = []


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


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Node~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def node(args = None):
    global vertices_, edges_

    # rclpy.init_node('hall_graph_pub', anonymous=True)
    rclpy.init(args=args)
    node = rclpy.create_node('hall_graph_pub')

    rate = node.create_rate(1)

    camera_type = node.declare_parameter('~camera_type', 'rgbd').value

    node.create_subscription(Float64MultiArray, camera_type + "/vertex_list", vertexCallBack,10)
    node.create_subscription(Float64MultiArray, camera_type + "/edge_list", edgesCallBack,10)

    marker_graph_pub_ = node.create_publisher(MarkerArray, 'marker_G', 10)

    tfBuffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tfBuffer,node)

    map_ = Map(node)

    # Get tf from camera link to base frame
    cond = 0
    while cond == 0:
        rclpy.spin_once(node)
        try:
            # (t_camera_base, q_camera_base) = tfBuffer.lookup_transform("base_link", "camera_link",
            #                                                              rclpy.time.Time())
            # map_.setCameraBaseTf(t_camera_base, q_camera_base)
            transformObject = tfBuffer.lookup_transform("base_link", "camera_link",
                                                                         rclpy.time.Time())
            t_camera_base = transformObject.transform.translation
            q_camera_base = transformObject.transform.rotation
            map_.setCameraBaseTf(t_camera_base, q_camera_base)
            cond = 1
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            node.get_logger().error(node.get_name() + ": Could not get TF from base to camera link.")
            cond = 0

    node.get_logger().info(node.get_name() + ": Loop start.")
    while rclpy.ok():
        map_.setNodes(vertices_)
        map_.setEdges(edges_)
        marker_graph_pub_.publish(map_.getGraphAsMarkerArray(only_nodes=False, color=False))

    rate.sleep()


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Main~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if __name__ == '__main__':
    try:
        node()
    except rclpy.exceptions.ROSInterruptException:
        pass
