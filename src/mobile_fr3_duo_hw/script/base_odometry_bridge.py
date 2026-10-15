#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Publish the planar base joints and /odom from the real base's odometry.

    ros2 run mobile_fr3_duo_hw base_odometry_bridge.py

The robot description is rooted at ``odom``, so the planar joints ARE ``odom -> base_link``,
which robot_state_publisher publishes; this node is their only publisher. The input is the
swerve controller's odometry. The base pose of the first sample becomes the ``odom`` origin, so
the joints start at zero and do not jump when the controller activates. The joints are published
from startup, at zero until the first sample, because MoveIt's robot state is incomplete
without them. The same pose goes to Nav2 on /odom, with the twist expressed in ``base_link``.
"""

import math

JOINT_NAMES = ("planar_x", "planar_y", "planar_theta")


def wrap(angle):
    """Wrap an angle into [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q):
    """Yaw of a geometry_msgs Quaternion, in [-pi, pi]."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def relative_pose(origin, pose):
    """Express a planar pose (x, y, yaw) in the frame of another planar pose."""
    ox, oy, oyaw = origin
    x, y, yaw = pose
    cos_o, sin_o = math.cos(oyaw), math.sin(oyaw)
    dx, dy = x - ox, y - oy
    return cos_o * dx + sin_o * dy, -sin_o * dx + cos_o * dy, wrap(yaw - oyaw)


def twist_in_base(vx, vy, yaw):
    """Rotate a planar velocity along the odometry axes into the base frame at heading yaw."""
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return vx * cos_yaw + vy * sin_yaw, -vx * sin_yaw + vy * cos_yaw


def main():
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    class BaseOdometryBridge(Node):
        def __init__(self):
            super().__init__("base_odometry_bridge")
            p = self.declare_parameter
            odometry_topic = p("odometry_topic", "/base_swerve_controller/odom").value
            self._odom_frame = p("odom_frame", "odom").value
            self._base_frame = p("base_frame", "base_link").value
            # The swerve controller's closed-loop twist is along its odometry axes; see the README.
            self._twist_in_odom_axes = p("twist_in_odom_axes", True).value
            rate = p("joint_state_rate", 50.0).value
            self._origin = None
            self._pose = (0.0, 0.0, 0.0)
            self._joints = self.create_publisher(JointState, "/joint_states", 10)
            self._odom = self.create_publisher(
                Odometry, p("odom_topic", "/odom").value, 10
            )
            self.create_subscription(Odometry, odometry_topic, self._on_odometry, 10)
            self.create_timer(1.0 / rate, self._publish_joints)

        def _publish_joints(self):
            joints = JointState()
            # MoveIt judges state freshness by the stamp.
            joints.header.stamp = self.get_clock().now().to_msg()
            joints.name = list(JOINT_NAMES)
            joints.position = list(self._pose)
            self._joints.publish(joints)

        def _on_odometry(self, msg):
            pose = msg.pose.pose
            yaw = yaw_from_quaternion(pose.orientation)
            absolute = (pose.position.x, pose.position.y, yaw)
            if self._origin is None:
                self._origin = absolute
                self.get_logger().info(
                    "First base odometry sample: it is the odom origin from now on."
                )
            self._pose = relative_pose(self._origin, absolute)
            x, y, rel_yaw = self._pose

            odom = Odometry()
            odom.header.stamp = msg.header.stamp
            odom.header.frame_id = self._odom_frame
            odom.child_frame_id = self._base_frame
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation.z = math.sin(rel_yaw / 2.0)
            odom.pose.pose.orientation.w = math.cos(rel_yaw / 2.0)
            odom.pose.covariance = msg.pose.covariance
            linear = msg.twist.twist.linear
            if self._twist_in_odom_axes:
                vx, vy = twist_in_base(linear.x, linear.y, yaw)
            else:
                vx, vy = linear.x, linear.y
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = msg.twist.twist.angular.z
            odom.twist.covariance = msg.twist.covariance
            self._odom.publish(odom)

    rclpy.init()
    node = BaseOdometryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
