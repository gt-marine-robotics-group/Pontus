#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from typing import Optional, List


class CameraRepublish(Node):
    """
    Node that republishes camera data to give QOS of Best Effort
    """
    def __init__(self):
        super().__init__('camera_republish')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.pub = self.create_publisher(
            Image, 'output', qos_profile=qos_profile)

        self.sub = self.create_subscription(Image, 'input', self.callback, 10)

    def callback(self, msg: Image) -> None:
        """
        Republish image from topic.

        Args:
        ----
        msg (Image): image we want to republish

        Return:
        ------
        None

        """
        self.pub.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)

    camera_republish = CameraRepublish()

    rclpy.spin(camera_republish)

    camera_republish.destroy_node()
    rclpy.shutdown()
