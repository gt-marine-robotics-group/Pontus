#!/usr/bin/env python3
import rclpy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros

class OdometryBroadcaster(Node):
    def __init__(self):
        super().__init__('odometry_broadcaster')
        self.br = TransformBroadcaster(self)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odom_callback,
            10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def odom_callback(self, msg: Odometry):
        transform = TransformStamped()

        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z

        transform.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
