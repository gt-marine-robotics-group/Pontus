#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class IMURepublishNode(Node):

    def __init__(self):
        super().__init__('imu_republish')

        self.pub = self.create_publisher(Imu, '/pontus/imu_0', 10)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            qos_profile=qos_profile
        )

    def imu_callback(self, msg):
        self.imu_data = msg
        self.imu_data.header.frame_id = 'imu_frame'
        self.imuPublisher.publish(self.imu_data)


def main(args=None):
    rclpy.init(args=args)

    node = IMURepublishNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()