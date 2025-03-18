import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from typing import List, Optional


class CameraInfoPublisherRight(Node):
    def __init__(self):
        super().__init__('camera_info_publisher_right')
        self.publisher_ = self.create_publisher(CameraInfo, '/pontus/camera_3/camera_info', 10)
        self.timer = self.create_timer(1.0, self.publish_camera_info)  # Publish every second
        self.get_logger().info("Camera Info Publisher Node Started")

    def publish_camera_info(self) -> None:
        """
        Publish camera info for right camera.

        This is temporary.
        """
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'narrow_stereo_right'
        msg.width = 640
        msg.height = 480
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.375589, 0.283808, 0.000384, 0.010389, 0.000000]
        msg.k = [611.48004, 0.0, 330.19915,
                 0.0, 609.85992, 266.5604,
                 0.0, 0.0, 1.0]
        msg.r = [0.9943939, 0.10204787, -0.02769503,
                 -0.10231248, 0.99471761, -0.00830824,
                 0.02670089, 0.01109521, 0.99958189]
        msg.p = [926.02054, 0.0, 350.00513, -97.09433,
                 0.0, 926.02054, 250.5993, 0.0,
                 0.0, 0.0, 1.0, 0.0]

        self.publisher_.publish(msg)
        self.get_logger().info("Published Camera Info")


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = CameraInfoPublisherRight()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
