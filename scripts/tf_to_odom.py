#!/usr/bin/env python3
"""
TF → Odometry bridge.
Listens to /tf for the odom→base_footprint transform and republishes as /odometry.

Usage:
    ros2 run sentry_decision tf_to_odom.py

    # Optional: override frame names
    ros2 run sentry_decision tf_to_odom.py --ros-args \
        -p odom_frame:=odom \
        -p base_frame:=base_footprint
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class TfToOdom(Node):
    def __init__(self):
        super().__init__("tf_to_odom")

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("rate_hz", 10.0)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        rate = self.get_parameter("rate_hz").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_pub = self.create_publisher(Odometry, "odometry", 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"Publishing /odometry from TF {self.odom_frame}→{self.base_frame} @ {rate} Hz"
        )

    def tick(self):
        try:
            t: TransformStamped = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time()
            )
            msg = Odometry()
            msg.header.stamp = t.header.stamp
            msg.header.frame_id = self.odom_frame
            msg.child_frame_id = self.base_frame
            msg.pose.pose.position.x = t.transform.translation.x
            msg.pose.pose.position.y = t.transform.translation.y
            msg.pose.pose.position.z = t.transform.translation.z
            msg.pose.pose.orientation = t.transform.rotation
            self.odom_pub.publish(msg)
        except Exception:
            pass  # TF not available yet, silently skip


def main():
    rclpy.init()
    node = TfToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
