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
        self.imu_data.header.frame_id = "imu_0"
        # #self.imu_data.header.frame_id = 'base_link'
        # #arr = [0.5429499344001188, -9.795323003681169, -10.006557007372148, -0.003681358719075772, -0.002156522807186763, 0.010213591866222437]

        # arr = [0.3, 0.0, -0.14, 0, 0, 0]
        # self.imu_data.linear_acceleration.x -=  arr[0]
        # self.imu_data.linear_acceleration.y -= arr[1]
        # self.imu_data.linear_acceleration.z -= arr[2]
        # #self.imu_data.linear_acceleration.z += 0.006225544166773
        # self.imu_data.angular_velocity.x -= arr[3]
        # self.imu_data.angular_velocity.y -= arr[4]
        # self.imu_data.angular_velocity.z -= arr[5]
        
        
        # x = self.imu_data.linear_acceleration.x
        # y = self.imu_data.linear_acceleration.y
        # z = self.imu_data.linear_acceleration.z

        # x_a = self.imu_data.angular_velocity.x
        # y_a = self.imu_data.angular_velocity.y
        # z_a = self.imu_data.angular_velocity.z

        # if abs(x) < 0.05:
        #     self.imu_data.linear_acceleration.x = 0.0 
        # if abs(y) < 0.05:
        #     self.imu_data.linear_acceleration.y = 0.0 
        # if abs(z) < 0.05:
        #     self.imu_data.linear_acceleration.z = 0.0 
        # if abs(x_a) < 0.05:
        #     self.imu_data.angular_velocity.x = 0.0
        # if abs(y_a) < 0.05:
        #     self.imu_data.angular_velocity.y = 0.0
        # if abs(z_a) < 0.05:
        #     self.imu_data.angular_velocity.z = 0.0        
        #self.imu_data.linear_acceleration.z = 0.0
        #self.imu_data.linear_acceleration.x = 0.0
        #self.imu_data.linear_acceleration.y = 0.0

        #self.imu_data.angular_velocity.x = 0.0
        #self.imu_data.angular_velocity.y = 0.0
        #self.imu_data.angular_velocity.z = 0.0

        #self.imu_data.orientation.x = 0.0
        #self.imu_data.orientation.y = 0.0
        #self.imu_data.orientation.z = 0.0
        #self.imu_data.orientation.w = 1.0
        #self.imu_data = Imu()

        self.pub.publish(self.imu_data)


def main(args=None):
    rclpy.init(args=args)

    node = IMURepublishNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
