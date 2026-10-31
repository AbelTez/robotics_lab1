#!/usr/bin/env python3
"""Drive the robot around a square: four times a straight side, then a quarter turn."""

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate


class SquareDriver(Node):

    def __init__(self):
        super().__init__("square_driver")
        self.declare_parameter("side_length", 0.5)

        self.drive_client = ActionClient(self, DriveDistance, "drive_distance")
        self.rotate_client = ActionClient(self, Rotate, "rotate")
        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)

    def stop(self):
        self.cmd_vel.publish(Twist())

    def send_goal(self, client, goal):
        """Send one goal and wait for its result, so the goals run one after another."""
        client.wait_for_server()
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def run(self):
        side_length = self.get_parameter("side_length").value

        for side in range(1, 5):
            self.get_logger().info(f"Side {side}: driving {side_length:.2f} m")
            goal = DriveDistance.Goal()
            goal.distance = float(side_length)
            if not self.send_goal(self.drive_client, goal):
                self.get_logger().error(f"Side {side}: drive_distance failed, stopping")
                self.stop()
                return

            self.get_logger().info(f"Side {side}: turning {math.pi / 2:.3f} rad")
            goal = Rotate.Goal()
            goal.angle = math.pi / 2
            if not self.send_goal(self.rotate_client, goal):
                self.get_logger().error(f"Side {side}: rotate failed, stopping")
                self.stop()
                return

        self.get_logger().info("Square complete")


def main():
    rclpy.init()
    node = SquareDriver()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
