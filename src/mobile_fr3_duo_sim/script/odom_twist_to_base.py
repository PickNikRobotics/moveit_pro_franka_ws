#!/usr/bin/env python3

"""Republishes the MuJoCo odometry with its twist in the child frame.

The plugin reports the base velocity in the odom frame, but nav_msgs/Odometry defines the
twist in child_frame_id (base_link), which is what Nav2's controllers read as the robot's
current velocity. Only the linear part needs rotating: the base is planar, so the angular
velocity is about the shared z axis.

The yaw comes from the same message's pose, so the rotation always matches the velocity it
corrects.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class OdomTwistToBase(Node):
    """Rotates the odometry twist from the odom frame into the child frame."""

    def __init__(self):
        super().__init__("odom_twist_to_base")

        self.declare_parameter("input_topic", "/mujoco/odom")
        self.declare_parameter("output_topic", "/odom")

        self._pub = self.create_publisher(
            Odometry, self.get_parameter("output_topic").value, 10
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("input_topic").value,
            self._on_odom,
            qos_profile_sensor_data,
        )

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        vx, vy = msg.twist.twist.linear.x, msg.twist.twist.linear.y
        msg.twist.twist.linear.x = vx * cos_yaw + vy * sin_yaw
        msg.twist.twist.linear.y = -vx * sin_yaw + vy * cos_yaw
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTwistToBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
