#!/usr/bin/env python3

# jplaced
# 2022, Universidad de Zaragoza

# This node initializes the frontier detectors with a pre defined starting map
# size.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import rclpy
import os
import sys
import time

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

MAP_SIZE = 25


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def init_point_selector(i):
    points = [[-MAP_SIZE, MAP_SIZE], [-MAP_SIZE, -MAP_SIZE], [MAP_SIZE, -MAP_SIZE], [MAP_SIZE, MAP_SIZE],
              [0.140955, -0.0519512]]

    five_points = []
    for (x, y) in points:
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = 0.0
        five_points.append(p)

    init_points = Marker()
    init_points.header.frame_id = "map"
    init_points.header.stamp = rclpy.clock.Clock().now().to_msg()
    init_points.points = five_points[0:i]

    return init_points


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Node~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def node(args=None):
    # rclpy.init_node('initializer', anonymous=False)
    rclpy.init(args=args)
    node = rclpy.create_node('point_init')

    # rate_hz = node.declare_parameter('~rate', 1)
    rate_hz = node.declare_parameter('~rate', 1).value
    rate = node.create_rate(1)
    # marker_pub = rclpy.Publisher('init_points', Marker, queue_size=5)
    marker_pub = node.create_publisher(Marker, 'init_points',5)

    i = 0
    while rclpy.ok() and i <= 5:
        if i > 5:
            i = 5
        init_points = init_point_selector(i)
        marker_pub.publish(init_points)
        i += 1
        # rate.sleep()
        time.sleep(1)

    node.get_logger().info(node.get_name() + ": Shutting down node.")
    try:
        os.system("rosnode kill /point_init")
    except KeyError:
        pass


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Main~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if __name__ == '__main__':
    try:
        node()
    except rclpy.exceptions.ROSInterruptException:
        pass
