#!/usr/bin/env python3

# jplaced
# 2022, Universidad de Zaragoza

# This node is a frontier detector, but it is not based on RRT. This node uses
# OpenCV tools to detect frontier points. It is intended to be run alone, and in
# multi-robot configuration only one instance should be run (running additional
# instances of this node does not make any difference). All detectors will be
# publishing detected frontier points on the same topic ("/detected_points").

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import rclpy
import cv2
import numpy as np
import sys

from copy import copy, deepcopy

from Functions import createMarker

from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, Point
import time

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Callbacks~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
map_data_ = OccupancyGrid()


def mapCallBack(data):
    global map_data_
    map_data_ = data


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# OpenCV frontier detector based on Canny algorithm.
def findFrontiers(map_data_,node):
    frontier_points = []

    data = map_data_.data
    w = map_data_.info.width
    h = map_data_.info.height
    resolution = map_data_.info.resolution
    X_origin = map_data_.info.origin.position.x
    Y_origin = map_data_.info.origin.position.y

    # Convert occupancy grid map to image
    img = np.zeros((h, w, 1), np.uint8)
    for i in range(0, h):
        for j in range(0, w):
            # occupied cell
            if data[i * w + j] == 100:
                img[i, j] = 0  # black
            # free cell
            elif data[i * w + j] == 0:
                img[i, j] = 255  # yellow
            # unknown cell
            elif data[i * w + j] == -1:
                img[i, j] = 205  # green
    o = cv2.inRange(img, 0, 1)
    # node.get_logger().info(str(w))

    # Plot image
    # img_plot = img[:,:,0]
    # plt.imshow(img_plot); plt.draw(); plt.pause(0.001);

    # Finds contours of the source image and draw them
    contours = cv2.findContours(o, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cv2.drawContours(o, contours, -1, (255, 255, 255), 5)
    o = cv2.bitwise_not(o)

    # Finds edges and overlaps them with the contours
    edges = cv2.Canny(img, 0, 255)
    res = cv2.bitwise_and(o, edges)

    # Finds contours on the overlapped image and draw them
    frontier = deepcopy(res)
    contours = cv2.findContours(frontier, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cv2.drawContours(frontier, contours, -1, (255, 255, 255), 2)
    # node.get_logger().info(str(frontier))

    # Get the final contours
    contours = cv2.findContours(frontier, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
    # node.get_logger().info(str(contours))

    if len(contours) > 0:
        for i in range(0, len(contours)):
            c = contours[i]
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius > 5:
                # Get the contours centers
                M = cv2.moments(c)
                center = (int(M["m10"] // M["m00"]), int(M["m01"] // M["m00"]))

                xr = center[0] * resolution + X_origin
                yr = center[1] * resolution + Y_origin
                pt = [np.array([xr, yr])]

                frontier_points = np.vstack([frontier_points, pt]) if len(frontier_points) > 0 else pt

                # Plot encircled frontiers
                # res = cv2.circle(res,(int(center[0]),int(center[1])),int(radius),75,2)
                # plt.imshow(res); plt.draw(); plt.pause(0.001);

    

    return frontier_points


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Node~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def node(args=None):
    global map_data_

    # rclpy.init_node('detector', anonymous=False)
    rclpy.init(args=args)
    # rclpy.init(args=sys.argv)
    node = rclpy.create_node('opencv_detector')
    node.get_logger().info("Run node")

    map_topic = node.declare_parameter('~map_topic', '/map').value

    # node.create_subscription(OccupancyGrid, "/GridMapper_node/rectified_map",  mapCallBack , 10)
    node.create_subscription(OccupancyGrid, "map",  mapCallBack , 10)
    targets_pub = node.create_publisher(PointStamped, 'detected_points', 10)
    pub = node.create_publisher(Marker,'opencv_shapes',  10)

    # Wait until map is received
    while len(map_data_.data) < 1:  # or mapData.header.seq<1
        rclpy.spin_once(node)
        pass

    param_1 = node.declare_parameter('~rate', 25).value
    rate = node.create_rate(param_1)

    exploration_goal = PointStamped()
    points = createMarker(frame=map_data_.header.frame_id, ns="markers", colors=[1.0, 1.0, 0.0],node=node)

    # while not rclpy.is_shutdown():
    while rclpy.ok():
        rclpy.spin_once(node)
        frontiers = findFrontiers(map_data_,node)
        pp = []
        temp_point = Point()
        temp_point.z = 0.0
        for i in frontiers:
            # Publish point to filter
            exploration_goal.header.frame_id = map_data_.header.frame_id
            exploration_goal.header.stamp = node.get_clock().now().to_msg()
            exploration_goal.point.x = i[0]
            exploration_goal.point.y = i[1]
            exploration_goal.point.z = float(0)
            targets_pub.publish(exploration_goal)

            # Marker to visualize in RViZ
            temp_point.x = i[0]
            temp_point.y = i[1]
            pp.append(copy(temp_point))

            # if points.id >= 50:
            #     points.action = Marker.DELETE
            #     points.id = 0
            # else:
            #     points.action = Marker.ADD
            #     points.id +=1
            # points.points = [exploration_goal.point]
            # pub.publish(points)

        # Publish marker to visualize in RViZ
        points.id += 1
        points.points = pp
        pub.publish(points)

        # rate.sleep()
        time.sleep(1)


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Main~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if __name__ == '__main__':
    try:
        node()
    except rclpy.exceptions.ROSInterruptException:
        pass
