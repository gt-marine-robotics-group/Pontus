import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from typing import Optional, List


class Bag2Mp4(Node):
    def __init__(self):
        super().__init__('bag_to_mp4')

        self.camera_sub = self.create_subscription(
            Image,
            '/pontus/camera_2/raw_image/compressed',
            self.camera_callback,
            10
        )
        self.output = "output.mp4"
        self.frame_width = 640
        self.frame_height = 480
        fps = 30
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = cv2.VideoWriter(self.output, fourcc, fps, (self.frame_width, self.frame_height))
        self.bridge = CvBridge()

    def camera_callback(self, msg: Image) -> None:
        """
        Handle camera callback and write to output.

        Args:
        ----
        msg (Image): the camera iamge

        Return:
        ------
        None

        """
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.out.write(cv_image)

    def destroy_node(self) -> None:
        """
        Override destroy_node to handle releasing the recording.

        Args:
        ----
        None

        Return:
        ------
        None

        """
        self.out.release()  # Properly release the VideoWriter
        super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = Bag2Mp4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
