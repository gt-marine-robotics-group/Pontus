import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from typing import List, Optional


class CompressedImageRepublisher(Node):
    def __init__(self):
        super().__init__('compressed_image_republisher')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            CompressedImage, '/pontus/camera_2/image_raw/compressed', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/pontus/camera_2/image_raw', 10)

        self.subscription2 = self.create_subscription(
            CompressedImage, '/pontus/camera_3/image_raw/compressed', self.image_callback2, 10)
        self.publisher2 = self.create_publisher(Image, '/pontus/camera_3/image_raw', 10)

    def image_callback(self, msg: CompressedImage) -> None:
        """
        Take a compressed image and republishes it into a regular image.

        This is useful for replaying bags.

        Args:
        ----
        msg (CompressedImage): image message from topic

        Return:
        ------
            None

        """
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            ros_image.header = msg.header
            self.publisher.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def image_callback2(self, msg: CompressedImage) -> None:
        """
        Take a compressed image and republishes it into a regular image.

        This is useful for replaying bags.

        Args:
        ----
        msg (CompressedImage): image message from topic

        Return:
        ------
            None

        """
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            ros_image.header = msg.header
            self.publisher2.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = CompressedImageRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
