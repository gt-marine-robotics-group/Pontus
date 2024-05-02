#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class OAKIMURepublishNode(Node):

    def __init__(self):
        super().__init__('imu_republish')

        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(
            Imu,
            '/oak/imu/data',
            self.imu_callback,
            qos_profile=qos_profile
        )

    def imu_callback(self, msg):
        self.imu_data = msg
        # self.imu_data.header.frame_id = 'sensor'
        self.imu_data.linear_acceleration.x += 1.69
        self.imu_data.linear_acceleration.y += 0.4
        self.imu_data.linear_acceleration.z += 1.45

        x = self.imu_data.linear_acceleration.x
        y = self.imu_data.linear_acceleration.y
        z = self.imu_data.linear_acceleration.z
       
        self.imu_data.linear_acceleration.z = -x
        self.imu_data.linear_acceleration.x = z
        self.imu_data.linear_acceleration.y = y

        if abs(self.imu_data.linear_acceleration.x) < 0.1:
            self.imu_data.linear_acceleration.x = 0.0
        if abs(self.imu_data.linear_acceleration.y) < 0.1:
            self.imu_data.linear_acceleration.y = 0.0
        if abs(9.81 - self.imu_data.linear_acceleration.x) < 0.1:
            self.imu_data.linear_acceleration.x = 9.81

        self.imu_data.header.frame_id = 'base_link'

        self.pub.publish(self.imu_data)


def main(args=None):
    rclpy.init(args=args)

    node = OAKIMURepublishNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
