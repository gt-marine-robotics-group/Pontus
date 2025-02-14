#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math
from tf_transformations import quaternion_from_euler
from sensor_msgs.msg import Image

class TfPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')

        # Create a TransformBroadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer to call the publish_tf function every 0.1 seconds
        self.camera_sub = self.create_subscription(
            Image,
            '/pontus/camera_2/image_raw',
            self.publish_tf,
            10
        )
        
        self.start_time = self.get_clock().now()

    def publish_tf(self, msg):
        # Create a TransformStamped object
        transform = TransformStamped()

        # Set the header information
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'base_link'  # Parent frame
        transform.child_frame_id = 'pontus/base_link/camera_2'  # Child frame

        # Set the transform (translation and rotation)
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 1.0  # Example translation

        # For rotation, use Quaternion (x, y, z, w)
        roll = -math.pi/2
        pitch = 0.0
        yaw = 0.0

        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    tf_publisher = TfPublisher()
    rclpy.spin(tf_publisher)
    tf_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
