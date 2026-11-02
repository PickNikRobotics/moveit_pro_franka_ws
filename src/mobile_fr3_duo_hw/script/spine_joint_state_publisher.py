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

"""Publish the spine joint on /joint_states from the Franka spine server's position topic.

    ros2 run mobile_fr3_duo_hw spine_joint_state_publisher.py

The spine has no ros2_control interface, so joint_state_broadcaster never reports it; this node
is its only publisher. It publishes nothing until the first valid sample: an unknown height is
not zero, and a wrong spine height moves both arms in the planning scene. After that it repeats
the last valid height at a fixed rate and holds it through invalid samples.
"""

JOINT_NAME = "franka_spine_vertical_joint"


class SpineHeight:
    """The last valid spine height, or None before the first one."""

    def __init__(self):
        self.position = None

    def update(self, position, valid):
        """Take one sample: a valid one replaces the height, an invalid one keeps it."""
        if valid:
            self.position = position


def main():
    import rclpy
    from franka_spine_msgs.msg import SpinePosition
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    class SpineJointStatePublisher(Node):
        def __init__(self):
            super().__init__("spine_joint_state_publisher")
            p = self.declare_parameter
            position_topic = p("position_topic", "/franka_spine_node/position").value
            rate = p("publish_rate", 50.0).value
            self._height = SpineHeight()
            self._valid = None
            self._pub = self.create_publisher(JointState, "/joint_states", 10)
            self.create_subscription(
                SpinePosition, position_topic, self._on_position, 10
            )
            self.create_timer(1.0 / rate, self._publish)

        def _on_position(self, msg):
            self._height.update(msg.position, msg.valid)
            # Log only when the sample health changes.
            if msg.valid != self._valid:
                self._valid = msg.valid
                if msg.valid:
                    self.get_logger().info(f"Spine height read: {msg.position:.4f} m")
                else:
                    self.get_logger().warning(
                        "Spine position sample invalid; holding the last valid height."
                    )

        def _publish(self):
            if self._height.position is None:
                return
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [JOINT_NAME]
            msg.position = [self._height.position]
            self._pub.publish(msg)

    rclpy.init()
    node = SpineJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
