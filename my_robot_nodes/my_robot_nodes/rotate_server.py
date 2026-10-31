#!/usr/bin/env python3
"""Action server that rotates the robot in place by a given angle.

The angle turned is measured by accumulating the yaw deltas between successive
odometry messages, so it keeps working when the yaw wraps at +-pi."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from my_robot_interfaces.action import Rotate


def yaw_from_quaternion(q):
    """Rotation about the vertical axis, in rad, of an orientation quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_difference(target, current):
    """target - current, wrapped to [-pi, pi] so the robot turns the short way."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


class RotateServer(Node):

    def __init__(self):
        super().__init__("rotate_server")
        self.declare_parameter("turn_speed", 0.5)

        self.yaw = None
        # Signed angle the robot has turned since the node started, in rad
        self.turned = 0.0

        # The goal runs in a loop, so the odometry callback needs its own thread
        # to keep updating while it does
        group = ReentrantCallbackGroup()
        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10, callback_group=group)
        self.action_server = ActionServer(
            self, Rotate, "rotate",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
            callback_group=group,
        )

    def on_odom(self, msg):
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.yaw is not None:
            self.turned += angle_difference(yaw, self.yaw)
        self.yaw = yaw

    def on_goal(self, goal):
        if goal.angle == 0.0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, goal_handle):
        goal = goal_handle.request.angle
        start = self.turned
        self.get_logger().info(f"Rotating {goal:.2f} rad")

        twist = Twist()
        twist.angular.z = math.copysign(self.get_parameter("turn_speed").value, goal)
        feedback = Rotate.Feedback()
        result = Rotate.Result()

        while rclpy.ok():
            turned = self.turned - start
            result.turned = turned

            if goal_handle.is_cancel_requested:
                self.cmd_vel.publish(Twist())
                goal_handle.canceled()
                self.get_logger().info(f"Canceled after {turned:.2f} rad")
                return result
            if (goal > 0.0 and turned >= goal) or (goal < 0.0 and turned <= goal):
                break

            self.cmd_vel.publish(twist)
            feedback.turned = turned
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)

        self.cmd_vel.publish(Twist())
        goal_handle.succeed()
        self.get_logger().info(f"Rotated {result.turned:.2f} rad")
        return result


def main():
    rclpy.init()
    node = RotateServer()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
