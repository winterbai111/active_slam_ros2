#!/usr/bin/env python3

# jplaced
# 2022, Universidad de Zaragoza

# Robot class

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import rclpy
import tf2_ros
# import actionlib
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose, NavigateToPose
import numpy as np

from scipy.spatial.transform import Rotation
from typing import Tuple

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Pose, PoseStamped

from Functions import yawBtw2Points

from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

import tf2_geometry_msgs
from action_msgs.msg import GoalStatus



# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Class~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
class Robot:
    def __init__(self, name: str, node:Node):
        """
        Constructor
        """
        # super().__init__('robot_node')
        self.node = node

        self.goal = MoveBaseGoal()
        self.start = PoseStamped()
        self.end = PoseStamped()
        self.pose = Pose()

        self.assigned_point = []
        self.name = name  # robot_1
        self.node.get_logger().info(self.node.get_name() + ': Robot Class started with robot name: ' + name)

        self.global_frame = self.node.declare_parameter('~global_frame', 'map').value
        self.robot_frame = self.node.declare_parameter('~robot_frame', 'base_link').value
        # self.listener = tf2_ros.TransformListener()
        self.tfBuffer_ = Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer_,self.node)
        # self.listener.waitForTransform(self.global_frame, self.robot_frame, rclpy.Time(0), rclpy.Duration(5))
        self.tfBuffer_.wait_for_transform_async(self.global_frame, self.robot_frame, rclpy.time.Time()) #, rclpy.duration.Duration(seconds=5))
        cond = 0
        while cond == 0:
            rclpy.spin_once(self.node)
            try:
                self.node.get_logger().info(self.node.get_name() + ': Robot Class is waiting for the robot transform.')
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
                # rclpy.logerr(tf2_ros.LookupException)
                self.node.get_logger().error('{}'.format(e))
                cond = 0
        self.node.get_logger().info(self.node.get_name() + ': Robot Class received the robot transform.')

        self.assigned_point = self.position
        # self.client = actionlib.SimpleActionClient(self.name + '/move_base', MoveBaseAction)
        # self.client.wait_for_server()

        # self.client = ActionClient(self.node, self.name + '/move_base', 'MoveBaseAction')
        # self.client.wait_for_server()
        self.client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        while not self.client.wait_for_server(timeout_sec=1.0):
                        node.get_logger().info(f"Waiting for navigation client to become available.")
        # self.client.wait_for_server()

        self.goal.target_pose.header.frame_id = self.global_frame
        self.goal.target_pose.header.stamp = self.node.get_clock().now().to_msg()

        # self.plan_service = self.node.declare_parameter('~plan_service', '/move_base_node/NavfnROS/make_plan').value
        # rclpy.wait_for_service(self.name + self.plan_service)
        # self.make_plan = rclpy.ServiceProxy(self.name + self.plan_service, GetPlan)

        self.plan_service = self.node.declare_parameter('~plan_service', '/planner/get_plan').value
        self._action_client = ActionClient(self.node, ComputePathToPose, '/compute_path_to_pose')

        self.start.header.frame_id = self.global_frame
        self.end.header.frame_id = self.global_frame
        self.node.get_logger().info(self.node.get_name() + ': Initialized robot.')

        self.node.get_logger().info(self.node.get_name() + ": Moving robot to +[0.1,0.1].")
        x = self.position[0] + 0.1
        y = self.position[1] + 0.1
        self.sendGoal([x, y])
        self.node.get_logger().info(self.node.get_name() + ": Moved robot to: [" + str(x) + ", " + str(y) + "].")

    def getPosition(self) -> np.array:
        """
        Gets robot's current position
        """
        cond = 0
        while cond == 0:
            try:
                transformObject = self.tfBuffer_.lookup_transform(self.global_frame, self.robot_frame, rclpy.time.Time())
                trans = transformObject.transform.translation
                rot = transformObject.transform.rotation
                self.position = np.array([trans.x, trans.y])
                cond = 1
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                cond = 0

        return self.position

    def getPose(self) -> Tuple[np.array, np.array]:
        """
        Gets robot's current pose as numpy arrays: position & quaternion
        """
        cond = 0
        while cond == 0:
            try:
                transformObject = self.tfBuffer_.lookup_transform(self.global_frame, self.robot_frame, rclpy.time.Time())
                trans = transformObject.transform.translation
                rot = transformObject.transform.rotation
                self.position = np.array([trans.x, trans.y])
                self.rotation = np.array([rot.x, rot.y, rot.z, rot.w])
                cond = 1
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                # rclpy.logerr(tf2_ros.LookupException)
                self.node.get_logger().error('{}'.format(e))
                cond = 0

        return self.position, self.rotation

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
                rclpy.logerr(tf2_ros.LookupException)
                self.node.get_logger().error('{}'.format(e))
                cond = 0

        return self.pose

    def sendGoal(self, point: np.array, waitGoalReached: bool = True):
        """
        Send goal and wait for task completion
        """
        self.goal.target_pose.pose.position.x = point[0]
        self.goal.target_pose.pose.position.y = point[1]
        current_pos = self.getPosition()
        goal_pos = [self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y]
        R_goal = Rotation.from_euler('xyz', [0., 0., yawBtw2Points(current_pos, goal_pos)], degrees=False)
        q_goal = R_goal.as_quat()
        self.goal.target_pose.pose.orientation.x = q_goal[0]
        self.goal.target_pose.pose.orientation.y = q_goal[1]
        self.goal.target_pose.pose.orientation.z = q_goal[2]
        self.goal.target_pose.pose.orientation.w = q_goal[3]

        # if waitGoalReached:
        #     self.client.send_goal_and_wait(self.goal)
        # else:
        #     self.client.send_goal(self.goal)
        # self.assigned_point = np.array(point)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goal.target_pose

        self.send_goal_future = self.client.send_goal_async(goal_msg)
        # Wait for the goal to complete
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.send_goal_future.done():
                self.goal_handle = self.send_goal_future.result()
                if self.goal_handle.accepted:
                    print("Goal accepted.")
                    self.result_future = self.goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self.node, self.result_future)
                    result = self.result_future.result().result
                    if result!=False:
                        # print("Goal execution done!")
                        self.node.get_logger().info("Goal execution done!")
                    else:
                        # print("Goal execution failed!")
                        self.node.get_logger().info("Goal execution failed!")
                    break
                else:
                    print("Goal rejected.")
                    break

    def sendGoalAsPose(self, pose: Pose, waitGoalReached: bool = True):
        """
        Send goal and wait for task completion
        """
        self.goal.target_pose.pose = pose

        # if waitGoalReached:
        #     self.client.send_goal_and_wait(self.goal)
        # else:
        #     self.client.send_goal(self.goal)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goal.target_pose

        self.send_goal_future = self.client.send_goal_async(goal_msg)
        # Wait for the goal to complete
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.send_goal_future.done():
                self.goal_handle = self.send_goal_future.result()
                if self.goal_handle.accepted:
                    print("Goal accepted.")
                    self.result_future = self.goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self.node, self.result_future)
                    result = self.result_future.result().result
                    if result!=False:
                        # print("Goal execution done!")
                        self.node.get_logger().info("Goal execution done!")
                    else:
                        # print("Goal execution failed!")
                        self.node.get_logger().info("Goal execution failed!")
                    break
                else:
                    print("Goal rejected.")
                    break

        self.assigned_point = np.array([pose.position.x, pose.position.y])

    def cancelGoal(self):
        self.node.get_logger().info(self.node.get_name() + ': Cancelling goal requested.')
        self.client.cancel_goal()
        self.assigned_point = self.getPosition()
        self.node.get_logger().info(self.node.get_name() + ': Goal cancelled.')

    def getState(self) -> int:
        """
        Returns status of goal
        http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
        :rtype: actionlib_msgs/GoalStatus
        """
        
        return self.result_future.result().status

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
        #                                self.start.header.stamp,
        #                                rclpy.duration.Duration(seconds=1))
        # start = tf2_geometry_msgs.do_transform_pose(self.start.pose, transform_start)

        # transform_end = self.tfBuffer_.lookup_transform(self.global_frame,
        #                                self.end.header.frame_id,
        #                                self.end.header.stamp,
        #                                rclpy.duration.Duration(seconds=1))
        # end = tf2_geometry_msgs.do_transform_pose(self.end.pose, transform_end)

        # plan = self.make_plan(start=start, goal=end, tolerance=0.1)

        # return plan.plan.poses
    
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
