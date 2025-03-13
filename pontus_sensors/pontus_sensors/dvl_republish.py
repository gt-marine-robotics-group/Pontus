#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf_transformations
from rclpy.qos import QoSProfile, ReliabilityPolicy

"""
Robot localization expects the position from odom to already be in the odom frame.
Since our DVL is not in the correct frame, this republisher will convert odom into the 
correct frame
TODO: Make the transforms based on urdf
"""

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
        """
        Transforms dvl frame of position to odom. Velocity is left alone because
        this is taken care of by robot localization.

        Parameters:
        msg (Odometry) : the Odometry from the dvk
        """
        # Roll 180 degrees to fix frame
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'dvl_a50_link'
        msg.pose.pose.position.y = -msg.pose.pose.position.y
        msg.pose.pose.position.z = -msg.pose.pose.position.z

        # Transform orientation
        r, p, y = tf_transformations.euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        roll_new = r
        pitch_new = -p
        yaw_new = -y
        q_new = tf_transformations.quaternion_from_euler(roll_new, pitch_new, yaw_new)

        msg.pose.pose.orientation.x = q_new[0]
        msg.pose.pose.orientation.y = q_new[1]
        msg.pose.pose.orientation.z = q_new[2]
        msg.pose.pose.orientation.w = q_new[3]

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    dvl_repub = DvlRepub()
    rclpy.spin(dvl_repub)
    rclpy.shutdown()

if __name__ == '__main__':
    main()