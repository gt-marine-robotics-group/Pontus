#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from dvl_msgs.msg import DLVDR
import tf_transformations
from rclpy.qos import QoSProfile, ReliabilityPolicy
from typing import Optional, List
import numpy as np

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
        self.create_subscription(
            Odometry,
            '/dvl/position',
            self.imu_callback,
            qos_profile=qos_profile
        )
        self.dvl_pub = self.create_publisher(
            Odometry,
            '/pontus/dvl',
            10
        )
        self.imu_pub = self.create_publisher(
            Imu,
            '/pontus/imu_dvl',
            10
        )

    def dvl_callback(self, msg: Odometry) -> None:
        """
        Transform dvl frame of position to odom.

        Velocity is left alone because this is taken care of by robot localization.

        Args:
        ----
        msg (Odometry): the Odometry from the dvl

        Return:
        ------
        None

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

        # Transform dvl (pretty sure this is already handled by the base_link to dvl frame tf
        # transform_x = 0.19685

        # msg.pose.pose.position.x += transform_x * np.cos(yaw_new)
        # msg.pose.pose.position.y += transform_x * np.sin(yaw_new)

        self.dvl_pub.publish(msg)

    def imu_callback(self, msg: DVLDR) -> None:
        """
        Pull the actual orientation of the dvl directly from it's IMU
        so we can use gravity to determine the absolute orientation of the vehicle

        Args:
        ----
        msg (Odometry): the Odometry from the dvl

        Return:
        ------
        None

        """
        imu_msg = Imu()

        imu_msg.header.frame_id = 'odom'
        imu_msg.child_frame_id = 'dvl_a50_link'

        # Transform orientation
        roll_new = np.deg2rad(msg.roll)
        pitch_new = np.deg2rad(-msg.pitch)
        yaw_new = np.deg2rad(-msg.yaw)
        q_new = tf_transformations.quaternion_from_euler(roll_new, pitch_new, yaw_new)

        imu_msg.pose.pose.orientation.x = q_new[0]
        imu_msg.pose.pose.orientation.y = q_new[1]
        imu_msg.pose.pose.orientation.z = q_new[2]
        imu_msg.pose.pose.orientation.w = q_new[3]

        self.imu_pub.publish(msg)

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    dvl_repub = DvlRepub()
    rclpy.spin(dvl_repub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
