#!/usr/bin/env python3

"""Turns the base-frame twist Nav2 publishes into world-frame planar joint velocities.

planar_x and planar_y are world-axis slides applied before the planar_theta hinge, while
cmd_vel is in base_link, so the linear part has to be rotated by the base yaw:

    planar_x_vel = vx * cos(yaw) - vy * sin(yaw)
    planar_y_vel = vx * sin(yaw) + vy * cos(yaw)
    planar_theta_vel = wz

The yaw is read from /joint_states rather than TF: odom -> base_link is published from the
same MuJoCo state a cycle later, so it lags while the base turns.

The velocity controller holds its last command, so the node publishes zeros when cmd_vel
goes quiet: a stopped publisher must stop the base, not leave it driving.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Must match the controller's joints, in order: the command is a bare array.
JOINTS = ["planar_x", "planar_y", "planar_theta"]


class BaseTwistToPlanar(Node):
    """Converts a base-frame twist into world-frame planar joint velocities."""

    def __init__(self):
        super().__init__("base_twist_to_planar")

        self.declare_parameter("twist_topic", "/cmd_vel")
        self.declare_parameter("command_topic", "/base_jgvc/commands")
        self.declare_parameter("yaw_joint", "planar_theta")
        self.declare_parameter("command_rate_hz", 50.0)
        self.declare_parameter("twist_timeout_sec", 0.5)

        self.yaw_joint = self.get_parameter("yaw_joint").value
        self.timeout = float(self.get_parameter("twist_timeout_sec").value)

        self._twist = None
        self._twist_at = None
        self._yaw = None

        self.create_subscription(
            Twist, self.get_parameter("twist_topic").value, self._on_twist, 10
        )
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, qos_profile_sensor_data
        )
        self._pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("command_topic").value, 10
        )
        self.create_timer(
            1.0 / float(self.get_parameter("command_rate_hz").value), self._tick
        )

    def _on_twist(self, msg):
        self._twist = msg
        self._twist_at = self.get_clock().now().nanoseconds

    def _on_joint_states(self, msg):
        if self.yaw_joint in msg.name:
            index = msg.name.index(self.yaw_joint)
            if index < len(msg.position):
                self._yaw = msg.position[index]

    def _tick(self):
        stale = (
            self._twist is None
            or (self.get_clock().now().nanoseconds - self._twist_at) / 1e9
            > self.timeout
        )
        command = Float64MultiArray()
        if stale or self._yaw is None:
            if self._yaw is None:
                self.get_logger().warn(
                    f"No {self.yaw_joint} in /joint_states yet; holding the base still.",
                    throttle_duration_sec=5.0,
                )
            command.data = [0.0, 0.0, 0.0]
        else:
            cos_yaw, sin_yaw = math.cos(self._yaw), math.sin(self._yaw)
            vx, vy = self._twist.linear.x, self._twist.linear.y
            command.data = [
                vx * cos_yaw - vy * sin_yaw,
                vx * sin_yaw + vy * cos_yaw,
                self._twist.angular.z,
            ]
        self._pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = BaseTwistToPlanar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
