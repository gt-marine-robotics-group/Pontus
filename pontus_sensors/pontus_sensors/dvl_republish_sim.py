#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from typing import Optional, List


class DvlRepubSim(Node):
    def __init__(self):
        super().__init__('dvl_repub')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.create_subscription(
            Odometry,
            '/dvl/odometry',
            self.dvl_callback,
            qos_profile=qos_profile
        )
        self.pub = self.create_publisher(
            Odometry,
            '/pontus/dvl',
            10
        )

    def dvl_callback(self, msg: Odometry) -> None:
        """
        Republish odometry with correct frame.

        This is primarily to ensure that the simulated odometry works with robot localiation.

        Args:
        ----
            msg (Odometry): the odometry from the topic

        Return:
        ------
            None

        """
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        self.pub.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    dvl_repub = DvlRepubSim()
    rclpy.spin(dvl_repub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
