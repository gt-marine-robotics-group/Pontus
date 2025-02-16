#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import tf_transformations
from rclpy.qos import QoSProfile, ReliabilityPolicy

class DvlRepub(Node):
    def __init__(self):
        super().__init__('dvl_repub')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.create_subscription(
            Odometry,
            '/dvl/odometry',
            self.dvl_callback,
            qos_profile=qos_profile
        )
        self.pub = self.create_publisher(
            Odometry,
            '/pontus/dvl',
            10
        )

    def dvl_callback(self, msg: Odometry):
        # Roll 180 degrees to fix frame
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'dvl_a50_link'
        msg.pose.pose.position.y = -msg.pose.pose.position.y
        msg.pose.pose.position.z = -msg.pose.pose.position.z
        q_rotate = tf_transformations.quaternion_from_euler(math.pi, 0, 0)
        q_new = tf_transformations.quaternion_multiply(q_rotate, [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

        msg.pose.pose.orientation.x = q_new[0]
        msg.pose.pose.orientation.y = q_new[1]
        msg.pose.pose.orientation.z = q_new[2]
        msg.pose.pose.orientation.w = q_new[3]

        msg.twist.twist.linear.y = -msg.twist.twist.linear.y
        msg.twist.twist.linear.z = -msg.twist.twist.linear.z
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    dvl_repub = DvlRepub()
    rclpy.spin(dvl_repub)
    rclpy.shutdown()

if __name__ == '__main__':
    main()