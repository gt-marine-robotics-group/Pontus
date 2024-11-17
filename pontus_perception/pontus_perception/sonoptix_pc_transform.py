#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import TransformStamped
from laser_geometry import LaserProjection
import tf2_ros
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs_py import point_cloud2
import tf_transformations 
import std_msgs.msg

class SonoptixPCTransform(Node):
    def __init__(self):
        super().__init__('sonoptix_cloud_transform')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, # keep the lastest 10
            depth=10  # max number of msg stored in a subscription
        )

        self.create_subscription(
            LaserScan,
            '/pontus/sonar_0',
            self.sonoptix_callback,
            qos_profile
        )

        self.transformed_pc_publisher = self.create_publisher(PointCloud2, '/pontus/sonar_0_transformed_pc', qos_profile)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.laser_projector = LaserProjection()


    def sonoptix_callback(self, msg: LaserScan):
        # Conver laser scan to point cloud
        point_cloud = self.laser_projector.projectLaser(msg)
        try:
            # Attempt to get transform and perform transform
            transform = self.tf_buffer.lookup_transform(
                'odom',
                msg.header.frame_id,
                rclpy.time.Time(), 
            )
            numpy_transformed_points = self.do_transform_cloud(point_cloud, transform)
            header = std_msgs.msg.Header()
            header.stamp = msg.header.stamp
            header.frame_id = 'odom'
            # Format transformed points into a point cloud
            cloud = point_cloud2.create_cloud_xyz32(header, numpy_transformed_points)
            self.transformed_pc_publisher.publish(cloud)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Failed to transform point cloud: {e}')

    
    def do_transform_cloud(self, point_cloud, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        rotation_matrix = tf_transformations.quaternion_matrix(quat)[:3, :3]
        pc_data = np.array(list(pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True)))
        transformed_points = []
        for point in pc_data:
            point_np_array = np.array([point[0], point[1], point[2]])
            # Rotate point
            rotated_point = np.dot(rotation_matrix, point_np_array)
            # Translate point
            transformed_point = rotated_point + np.array([translation.x, translation.y, translation.z])
            transformed_points.append(transformed_point)
        numpy_transformed_points = np.array(transformed_points)
        return numpy_transformed_points


def main(args=None):
    rclpy.init(args=args)
    node = SonoptixPCTransform()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
