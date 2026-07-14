#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from dvl_msgs.msg import DVLDR
import tf_transformations
from rclpy.qos import QoSProfile, ReliabilityPolicy
from typing import Optional, List
import numpy as np

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseStamped

"""
Robot localization expects the position from odom to already be in the odom frame.
Since our DVL is not in the correct frame, this republisher will convert odom into the
correct frame
TODO: Make the transforms based on urdf
"""


class DvlRepub(Node):
    def __init__(self):
        super().__init__('dvl_repub')

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
            '/dvl/odometry',
            self.dvl_callback,
            qos_profile=qos_profile
        )

        if self.imu_correction:
            self.imu_sub = self.create_subscription(
                Imu,
                # '/pontus/imu_0',
                '/imu/data',
                self.imu_callback,
                qos_profile=qos_profile
            )

        self.dvl_pub = self.create_publisher(
            Odometry,
            '/pontus/dvl',
            10
        )

        self.debug_pose_pub = self.create_publisher(
            PoseStamped,
            '/debug_IMU_POSE',
            10
        )

        self.dvl_msg = None
        self.imu_msg = None

        self.dvl_inialization_offset = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


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
        q_new = tf_transformations.quaternion_from_euler(
            roll_new, pitch_new, yaw_new)

        msg.pose.pose.orientation.x = q_new[0]
        msg.pose.pose.orientation.y = q_new[1]
        msg.pose.pose.orientation.z = q_new[2]
        msg.pose.pose.orientation.w = q_new[3]

        # Transform dvl (pretty sure this is already handled by the base_link to dvl frame tf
        # transform_x = 0.19685

        # msg.pose.pose.position.x += transform_x * np.cos(yaw_new)
        # msg.pose.pose.position.y += transform_x * np.sin(yaw_new)

        # TODO: Do we need to flip the dvl rotational velocities?

        self.dvl_msg = msg
        self.dvl_pub.publish(self.rotate_dvl_msg(msg))

    def imu_callback(self, msg: DVLDR) -> None:
        """
        Pull the actual orientation of the vehicle directly from the imu
        so we can determine the absolute orientation of the vehicle

        Args:
        ----
        msg (Imu): the Imu data

        Return:
        ------
        None

        """

        self.imu_msg = msg

        if self.dvl_msg:
            result = self.calculate_dvl_initialization_offset()
            if result:
                # Only calculate the offset once
                self.destroy_subscription(self.imu_sub)

    def calculate_dvl_initialization_offset(self):
        if self.dvl_inialization_offset is not None:
            return True

        try:
            t: TransformStamped = self.tf_buffer.lookup_transform(
                "imu_0",
                "base_link",
                rclpy.time.Time() # Empty time gets most recent tf
            )

        except TransformException as ex:
            return False

        r_odometry_reported_orientation = R.from_quat([
            self.dvl_msg.pose.pose.orientation.x,
            self.dvl_msg.pose.pose.orientation.y,
            self.dvl_msg.pose.pose.orientation.z,
            self.dvl_msg.pose.pose.orientation.w
        ])

        r_imu_mount_offset = R.from_quat([
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        ])

        r_imu_orientation = R.from_quat([
            self.imu_msg.orientation.x,
            self.imu_msg.orientation.y,
            self.imu_msg.orientation.z,
            self.imu_msg.orientation.w,
        ])
        euler_imu_orientation = r_imu_orientation.as_euler("xyz")

        r_imu_remapped = R.from_euler("xyz", [
            euler_imu_orientation[1],
            -euler_imu_orientation[0],
            euler_imu_orientation[2],
        ])


        r_vehicle_real_orientation = r_imu_mount_offset * r_imu_remapped

        # self.debug_pub(r_vehicle_real_orientation)

        # Odom to map frame transform to correct for odometry zeroing its
        # axes not level to the world
        # by rotating the reported odom data by the inverse of our actual orientation
        r_correction = r_odometry_reported_orientation * r_vehicle_real_orientation.inv()
        euler_correction = r_correction.as_euler("xyz")

        # Ignore yaw
        r_final = R.from_euler("xyz", [
            euler_correction[0],
            euler_correction[1],
            0.0
        ])
        final_quat = r_final.as_quat()
        self.dvl_inialization_offset = r_final

        # self.get_logger().info(f"imu mount offset: {r_imu_mount_offset.as_euler('xyz')}")
        # self.get_logger().info(f"imu base data: {r_imu_orientation.as_euler('xyz')}")
        # self.get_logger().info(f"imu remapped data: {r_imu_remapped.as_euler('xyz')}")
        # self.get_logger().info(f"vehicle real orientation: {r_vehicle_real_orientation.as_euler('xyz')}")
        # self.get_logger().info(f"odom orientation: {r_odometry_reported_orientation.as_euler('xyz')}")
        # self.get_logger().info(f"Correction: {r_correction.as_euler('xyz')}")
        self.get_logger().info(f"Setting DVL Initialization offset to {r_final.as_euler('xyz')}")
        return True

    def rotate_dvl_msg(self, msg: Odometry):
        if self.dvl_inialization_offset is None:
            return msg

        r_dvl_orientation = R.from_quat([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        r_corrected_orientation = r_dvl_orientation * self.dvl_inialization_offset
        corrected_quat = r_corrected_orientation.as_quat()

        vec_dvl_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        vec_corrected_position = self.dvl_inialization_offset.apply(vec_dvl_position)

        vec_dvl_velocity = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ]
        vec_corrected_velocity = self.dvl_inialization_offset.apply(vec_dvl_velocity)

        corrected_msg = Odometry()
        corrected_msg.pose.pose.position.x = vec_corrected_position[0]
        corrected_msg.pose.pose.position.y = vec_corrected_position[1]
        corrected_msg.pose.pose.position.z = vec_corrected_position[2]

        corrected_msg.pose.pose.orientation.x = corrected_quat[0]
        corrected_msg.pose.pose.orientation.y = corrected_quat[1]
        corrected_msg.pose.pose.orientation.z = corrected_quat[2]
        corrected_msg.pose.pose.orientation.w = corrected_quat[3]

        corrected_msg.twist.twist.linear.x = vec_corrected_velocity[0]
        corrected_msg.twist.twist.linear.y = vec_corrected_velocity[1]
        corrected_msg.twist.twist.linear.z = vec_corrected_velocity[2]

        return corrected_msg

    def debug_pub(self, rot):
        msg = PoseStamped()
        msg.header.frame_id = "map"


        quat = rot.as_quat()
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        self.debug_pose_pub.publish(msg)



def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    dvl_repub = DvlRepub()
    rclpy.spin(dvl_repub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
