import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from typing import Optional, List


class StereoDetection(Node):
    def __init__(self):
        super().__init__("stereo_detection")

        self.disparity_sub = self.create_subscription(
            Image,
            '/disparity_image',
            self.disparity_callback,
            10,
        )
        self.bridge = CvBridge()

    def disparity_callback(self, msg: Image) -> None:
        """
        Perform classification via the disparity image.

        This will use DBSCAN.
        """
        # disparity_image = self.bridge.imgmsg_to_cv2(msg)
        # TODO: Finish this
        pass


def main(args: Optional[List[str]] = None) -> None:
    rclpy.spin(args=args)
    node = StereoDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
