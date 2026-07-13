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

# IMPORTANT NOTES:
#   1. This node should not be used until the rest of the codebase
#       has started using the map frame and odom frame correctly
#       (including using the /pontus/odometry topic in the correct frames)
#
#   2. This node isn't working correctly yet, the offset it generates is not rotate correctly
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

        # Wait till we have imu and odom data coming in,
        # Then latch the most recent odometry and compare it to the faster
        # rate imu data
        self.imu: Imu = None
        self.odom: Odometry = None

        self.odom_transform = TransformStamped()
        self.transform_broadcaster = TransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def odom_callback(self, msg: Odometry) -> None:
        # Wait till we have imu data coming in to consider odom
        if not self.imu_correction or self.imu is not None:
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

        self.imu = msg

        # Wait till we have odom data to configure the transform
        if self.odom is None:
            return

        try:
            t: TransformStamped = self.tf_buffer.lookup_transform(
                "imu_0",
                "base_link",
                rclpy.time.Time() # Empty time gets most recent tf
            )

            # Odometry reported orientation
            r_odometry_reported_orientation = R.from_quat([
                self.odom.pose.pose.orientation.x,
                self.odom.pose.pose.orientation.y,
                self.odom.pose.pose.orientation.z,
                self.odom.pose.pose.orientation.w
            ])

            # Imu Mounting Offset
            r_imu_mount_offset = R.from_quat([
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ])

            # IMU Orientation Data
            euler_angles = tf_transformations.euler_from_quaternion([
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            ])
            r_orientation = R.from_euler("xyz", [
                euler_angles[0],
                euler_angles[1],
                euler_angles[2]
            ])

            r_vehicle_real_orientation = r_imu_mount_offset * r_orientation

            # Odom to map frame transform to correct for odometry zeroing its
            # axes not level to the world
            r_correction = (r_vehicle_real_orientation * r_odometry_reported_orientation).inv()
            euler_correction = r_correction.as_euler("xyz")

            # Ignore yaw
            r_final = R.from_euler("xyz", [
                euler_correction[0],
                euler_correction[1],
                0.0
            ])
            final_quat = r_final.as_quat()

            self.odom_transform.transform.rotation.x = final_quat[0]
            self.odom_transform.transform.rotation.y = final_quat[1]
            self.odom_transform.transform.rotation.z = final_quat[2]
            self.odom_transform.transform.rotation.w = final_quat[3]

            self.get_logger().info(f"Setting map->odom TF to {r_final.as_euler("xyz")}")
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
        if self.odom:
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