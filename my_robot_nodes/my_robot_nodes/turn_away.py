#!/usr/bin/env python3
"""Drive forward until an obstacle is too close, then turn away from it.

The node starts disabled. It can be switched on and off with /turn_away_enable,
and its turning speed can be changed with /set_turn_speed."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool

from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus
from my_robot_interfaces.srv import SetTurnSpeed


class TurnAway(Node):

    def __init__(self):
        super().__init__("turn_away")
        self.declare_parameter("forward_speed", 0.2)
        self.declare_parameter("turn_speed", 0.5)
        self.declare_parameter("max_turn_speed", 1.5)

        self.enabled = False

        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.status = self.create_publisher(RobotStatus, "robot_status", 10)
        self.create_subscription(ObstacleInfo, "obstacle", self.on_obstacle, 10)
        self.create_service(SetBool, "turn_away_enable", self.on_enable)
        self.create_service(SetTurnSpeed, "set_turn_speed", self.on_set_turn_speed)

    def on_enable(self, request, response):
        self.enabled = request.data
        if not self.enabled:
            self.cmd_vel.publish(Twist())
        response.success = True
        response.message = "Turn away enabled" if self.enabled else "Turn away disabled"
        self.get_logger().info(response.message)
        return response

    def on_set_turn_speed(self, request, response):
        max_turn_speed = self.get_parameter("max_turn_speed").value
        if request.turn_speed <= 0.0 or request.turn_speed > max_turn_speed:
            response.success = False
            response.message = f"turn_speed must be in (0, {max_turn_speed}]"
        else:
            self.set_parameters(
                [Parameter("turn_speed", Parameter.Type.DOUBLE, float(request.turn_speed))]
            )
            response.success = True
            response.message = f"turn_speed set to {request.turn_speed}"
        self.get_logger().info(response.message)
        return response

    def on_obstacle(self, msg):
        forward_speed = self.get_parameter("forward_speed").value
        turn_speed = self.get_parameter("turn_speed").value

        status = RobotStatus()
        status.obstacle_distance = msg.distance

        if not self.enabled:
            status.state = "stopped"
            self.status.publish(status)
            return

        twist = Twist()
        if not msg.too_close:
            twist.linear.x = forward_speed
            status.state = "forward"
        elif msg.angle > 0.0:
            # Obstacle on the left, turn right
            twist.angular.z = -turn_speed
            status.state = "turning"
        else:
            twist.angular.z = turn_speed
            status.state = "turning"
        self.cmd_vel.publish(twist)
        self.status.publish(status)


def main():
    rclpy.init()
    node = TurnAway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
