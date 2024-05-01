#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class DepthRepublishNode(Node):

    def __init__(self):
        super().__init__('depth_republish')

        self.pub = self.create_publisher(Odometry, '/pontus/depth_0', 10)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.depth_sub = self.create_subscription(
            Float32,
            '/depth_sensor',
            self.depth_callback,
            qos_profile=qos_profile
        )

    def depth_callback(self, msg):

        odom_msg = Odometry()
        odom_msg.header.frame_id = 'odom' # TODO: should this be odom or map
        odom_msg.pose.pose.position.z = -msg.data
        self.imuPublisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)

    node = DepthRepublishNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
