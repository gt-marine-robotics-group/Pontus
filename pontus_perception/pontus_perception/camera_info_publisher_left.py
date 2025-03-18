# From chatgpt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from typing import List, Optional


class CameraInfoPublisherLeft(Node):
    def __init__(self):
        super().__init__('camera_info_publisher_left')

        # Publisher for CameraInfo message
        self.publisher = self.create_publisher(CameraInfo, '/pontus/camera_2/camera_info', 10)

        # Set the camera calibration values
        self.camera_info = CameraInfo()

        self.camera_info.width = 640
        self.camera_info.height = 480
        self.camera_info.header.frame_id = 'narrow_stereo/left'

        # Camera Matrix (3x3)
        self.camera_info.k = [608.14976, 0.0, 318.879,
                              0.0, 606.49771, 247.27147,
                              0.0, 0.0, 1.0]

        # Distortion Coefficients (plumb_bob model)
        self.camera_info.d = [0.341522, 0.394931, -0.005284, -0.002347, 0.0]

        # Rectification Matrix (3x3)
        self.camera_info.r = [0.99195853, 0.1206723, -0.03816381,
                              -0.12029506, 0.99266517, 0.01203966,
                              0.03933674, -0.00735193, 0.99919896]

        # Projection Matrix (3x4)
        self.camera_info.p = [926.02054, 0.0, 350.00513, 0.0,
                              0.0, 926.02054, 250.5993, 0.0,
                              0.0, 0.0, 1.0, 0.0]

        # Timer to periodically publish the camera info
        self.timer = self.create_timer(0.2, self.publish_camera_info)

    def publish_camera_info(self) -> None:
        """
        Publish camera info message.

        This is temporary.
        """
        # Publish the CameraInfo message
        self.publisher.publish(self.camera_info)
        self.get_logger().info('Publishing CameraInfo message')


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    camera_info_publisher = CameraInfoPublisherLeft()

    rclpy.spin(camera_info_publisher)

    # Shutdown the node
    camera_info_publisher.destroy_node()
    rclpy.shutdown()
