#!/usr/bin/env python3

# jplaced
# 2022, Universidad de Zaragoza

# Robot class

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import rclpy
import tf2_ros
import numpy as np

import tf2_geometry_msgs

from typing import Tuple
# from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Pose, PoseStamped
from nav2_msgs.action import ComputePathToPose

from tf2_ros import Buffer, TransformListener
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Class~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
class Robot():
    def __init__(self, name: str, node:Node):
        # super().__init__('simple_robot_node')
        """
        Constructor
        """
        self.start = PoseStamped()
        self.end = PoseStamped()
        self.pose = Pose()
        self.node = node
        self.name = name  # robot_1
        self.node.get_logger().info(self.node.get_name() + ': Robot Class started with robot name: ' + name)

        # self.global_frame = rclpy.get_param('~global_frame', 'map')
        # self.robot_frame = rclpy.get_param('~robot_frame', 'base_footprint')
        self.global_frame = self.node.declare_parameter('~global_frame', "map").value
        self.robot_frame = self.node.declare_parameter('~robot_frame_', 'odom').value

        self.tfBuffer_ = Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer_,self.node)
        # self.listener.waitForTransform(self.global_frame, self.robot_frame, rclpy.Time(0), rclpy.Duration(5))
        self.tfBuffer_.wait_for_transform_async(self.global_frame, self.robot_frame, rclpy.time.Time()) #, rclpy.duration.Duration(seconds=5))
        cond = 0
        while cond == 0:
            rclpy.spin_once(self.node)
            try:
                # self.tfBuffer_.can_transform(self.global_frame, self.robot_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=2))
                self.node.get_logger().info(self.node.get_name() + ': Robot Class is waiting for the robot transform.')
                # (trans, rot) = self.listener.lookupTransform(self.global_frame, self.robot_frame, rclpy.Time(0))
                transformObject = self.tfBuffer_.lookup_transform(self.global_frame, self.robot_frame, rclpy.time.Time())
                trans = transformObject.transform.translation
                rot = transformObject.transform.rotation
                self.position = np.array([trans.x, trans.y])
                self.rotation = np.array([rot.x, rot.y, rot.z, rot.w])
                self.pose.position.x = trans.x
                self.pose.position.y = trans.y
                self.pose.position.z = float(0)
                self.pose.orientation.x = rot.x
                self.pose.orientation.y = rot.y
                self.pose.orientation.z = rot.z
                self.pose.orientation.w = rot.w
                cond = 1
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.node.get_logger().error('{}'.format(e))
                cond = 0
        self.node.get_logger().info(self.node.get_name() + ': Robot Class received the robot transform.')

        self.plan_service = self.node.declare_parameter('~plan_service', '/planner/get_plan').value
        # print(self.name + self.plan_service)

        # rclpy.wait_for_service(self.name + self.plan_service)
        # self.make_plan = rclpy.ServiceProxy(self.name + self.plan_service, GetPlan)

        self._action_client = ActionClient(self.node, ComputePathToPose, '/compute_path_to_pose')
        self._action_client.wait_for_server()

        self.start.header.frame_id = self.global_frame
        self.end.header.frame_id = self.global_frame
        self.node.get_logger().info(self.node.get_name() + ': Initialized robot.')

    def getPosition(self) -> np.array:
        """
        Gets robot's current position
        """
        cond = 0
        while cond == 0:
            rclpy.spin_once(self.node)
            try:
                transformObject = self.tfBuffer_.lookup_transform(self.global_frame, self.robot_frame, rclpy.time.Time())
                trans = transformObject.transform.translation
                rot = transformObject.transform.rotation
                self.position = np.array([trans.x, trans.y])
                cond = 1
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                cond = 0

        return self.position

    def getPoseAsGeometryMsg(self) -> Pose:
        """
        Gets robot's current pose as geometry_msgs/Pose
        """
        cond = 0
        while cond == 0:
            try:
                transformObject = self.tfBuffer_.lookup_transform(self.global_frame, self.robot_frame, rclpy.time.Time())
                trans = transformObject.transform.translation
                rot = transformObject.transform.rotation
                self.pose.position.x = trans.x
                self.pose.position.y = trans.y
                self.pose.position.z = float(0)
                self.pose.orientation.x = rot.x
                self.pose.orientation.y = rot.y
                self.pose.orientation.z = rot.z
                self.pose.orientation.w = rot.w
                cond = 1
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                # rclpy.logerr(tf2_ros.LookupException)
                self.node.get_logger().error('{}'.format(e))
                cond = 0

        return self.pose

    def makePlan(self, start: Pose, end: Pose) -> Tuple[PoseStamped]:
        """
        Returns poses in plan (nav_msgs/Path)
        :rtype: geometry_msgs/PoseStamped
        """
        # self.start.header.seq += 1
        self.start.header.stamp = self.node.get_clock().now().to_msg()
        self.start.pose.position.x = start.position.x
        self.start.pose.position.y = start.position.y
        self.start.pose.position.z = start.position.z
        self.start.pose.orientation.x = start.orientation.x
        self.start.pose.orientation.y = start.orientation.y
        self.start.pose.orientation.z = start.orientation.z
        self.start.pose.orientation.w = start.orientation.w

        # self.end.header.seq += 1
        self.end.header.stamp = self.node.get_clock().now().to_msg()
        self.end.pose.position.x = end.position.x
        self.end.pose.position.y = end.position.y
        self.end.pose.position.z = end.position.z
        self.end.pose.orientation.x = end.orientation.x
        self.end.pose.orientation.y = end.orientation.y
        self.end.pose.orientation.z = end.orientation.z
        self.end.pose.orientation.w = end.orientation.w

        # start = self.listener.transformPose(self.global_frame, self.start)
        # end = self.listener.transformPose(self.global_frame, self.end)

        # transform_start = self.tfBuffer_.lookup_transform(self.global_frame,
        #                                self.start.header.frame_id,
        #                                self.start.header.stamp,)
        #                             #    rclpy.duration.Duration(seconds=1))
        # start = tf2_geometry_msgs.do_transform_pose(self.start.pose, transform_start)

        # transform_end = self.tfBuffer_.lookup_transform(self.global_frame,
        #                                self.end.header.frame_id,
        #                                self.end.header.stamp,)
        #                             #    rclpy.duration.Duration(seconds=1))
        # end = tf2_geometry_msgs.do_transform_pose(self.end.pose, transform_end)

        # goal_msg = ComputePathToPose.Goal()
        # goal_msg.start.pose= self.start.pose
        # goal_msg.goal.pose = self.end.pose
        # goal_msg.use_start = True

        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().info("'ComputePathToPose' action server not available, waiting...")

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self.end
        goal_msg.start = self.start

        # self.node.get_logger().info('Getting path...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.node.get_logger().error('Get path was rejected!')
            return None

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, self.result_future)
        self.status = self.result_future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().warn('Getting path failed with status code: {0}'.format(self.status))
            return None

        return self.result_future.result().result.path.poses
        
        
        # plan = self._action_client.send_goal_async(goal_msg)
        # rclpy.spin_until_future_complete(self.node, plan)
        # plan = self.make_plan(start=start, goal=end, tolerance=0.1)

        # self.goal_handle = plan.result()
        # self.result_future = self.goal_handle.get_result_async()
        # rclpy.spin_until_future_complete(self.node, self.result_future)

        # return plan.path.poses
        # return self.result_future.result().result.path.poses

        # send_goal_future = self._action_client.send_goal_async(goal_msg)
        # rclpy.spin_until_future_complete(self.node, send_goal_future)
        # self.goal_handle = send_goal_future.result()

        # if not self.goal_handle.accepted:
        #     self.node.get_logger().error('Get path was rejected!')
        #     return None

        # self.result_future = self.goal_handle.get_result_async()
        # rclpy.spin_until_future_complete(self.node, self.result_future)
        # self.status = self.result_future.result().status

        # self.node.get_logger().info(str(self.result_future.result().result.path.poses))

        # return self.result_future.result().result.path.poses
