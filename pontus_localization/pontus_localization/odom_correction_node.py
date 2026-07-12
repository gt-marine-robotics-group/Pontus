#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf_transformations
from tf2_ros.transform_broadcaster import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy
from typing import Optional, List
import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R


class OdomCorrection(Node):
    def __init__(self):
        super().__init__('odom_correction')

        self.declare_parameter('imu_correction', True)
        self.declare_parameter('depth_correction', False)

        self.imu_correction = self.get_parameter('imu_correction').get_parameter_value().bool_value
        self.depth_correction = self.get_parameter('depth_correction').get_parameter_value().bool_value

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odom_callback,
            qos_profile=qos_profile
        )

        if self.imu_correction:
            self.imu_sub = self.create_subscription(
                Imu,
                '/pontus/imu_0',
                self.imu_callback,
                qos_profile=qos_profile
            )

        if self.depth_correction:
            self.depth_sub = self.create_subscription(
                Odometry,
                '/pontus/depth_0',
                self.depth_callback,
                qos_profile=qos_profile
            )

        # This is necessary to keep the TF from timing out
        self.tf_pub_timer = self.create_timer(0.05, self.publish_tf)

        self.odom: Odometry = None

        self.odom_transform = TransformStamped()
        self.transform_broadcaster = TransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def odom_callback(self, msg: Odometry) -> None:
        self.odom = msg

    def imu_callback(self, msg: Imu) -> None:
        """
        Pull the actual orientation of the sub directly from the IMU
        so we can use gravity to determine the absolute orientation of the vehicle
        to correct the odometry frame

        Args:
        ----
        msg (Imu): the data from the IMU

        Return:
        ------
        None

        """

        try:
            t: TransformStamped = self.tf_buffer.lookup_transform(
                    "imu_0",
                    "base_link",
                    Time.from_msg(msg.header.stamp)
                )

            imu_offset_quat = [
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ]
            r_imu_offset = R.from_quat(imu_offset_quat)

            orientation_quat = [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            ]

            euler_angles = tf_transformations.euler_from_quaternion(orientation_quat)
            r_orientation = R.from_euler("xyz", [
                euler_angles[0],
                euler_angles[1],
                0.0
            ])

            r_final = r_imu_offset * r_orientation

            odom_quat = r_final.as_quat()

            self.odom_transform.transform.rotation.x = odom_quat[0]
            self.odom_transform.transform.rotation.y = odom_quat[1]
            self.odom_transform.transform.rotation.z = odom_quat[2]
            self.odom_transform.transform.rotation.w = odom_quat[3]

            self.get_logger().info(f"Setting map->odom TF to {r_orientation.as_euler("xyz")}")
            self.destroy_subscription(self.imu_sub)
        except TransformException as ex:
            pass


    def depth_callback(self, msg: Odometry) -> None:
        """
        Use the depth sensor to make sure our dvl isn't drifting too far during the run

        Args:
        ----
        msg (Imu): the data from the IMU

        Return:
        ------
        None

        """

        # TODO: Actually implement this properly.
        # Probably will need the depth sensor and the dvl estimated position
        # of the sub so we can correct for drift
        self.odom_transform.transform.z = -msg.pose.position.z

    def publish_tf(self):
        if (self.odom):
            self.odom_transform.header.stamp = self.odom.header.stamp
            self.odom_transform.header.frame_id = "map"
            self.odom_transform.child_frame_id = "odom"
            self.transform_broadcaster.sendTransform(self.odom_transform)

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    odom_correction = OdomCorrection()

    try:
        rclpy.spin(odom_correction)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()