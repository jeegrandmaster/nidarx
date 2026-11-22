#!/usr/bin/env python3

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):

    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self._tf_broadcaster = TransformBroadcaster(self)
        self._odom_subscription = self.create_subscription(
            Odometry,
            '/nidarx_drone/odom',
            self._broadcast_odom_transform,
            10,
        )

    def _broadcast_odom_transform(self, odometry):
        if not odometry.header.frame_id or not odometry.child_frame_id:
            self.get_logger().warning('Ignoring odometry with an empty frame id')
            return

        transform = TransformStamped()
        transform.header = odometry.header
        transform.child_frame_id = odometry.child_frame_id
        transform.transform.translation.x = odometry.pose.pose.position.x
        transform.transform.translation.y = odometry.pose.pose.position.y
        transform.transform.translation.z = odometry.pose.pose.position.z
        transform.transform.rotation = odometry.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
