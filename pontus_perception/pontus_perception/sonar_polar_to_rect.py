import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import Image, CompressedImage
# import cv2
from cv_bridge import CvBridge
import numpy as np
# import open3d as o3d
from typing import Optional, List
import math


class SonarPolarToRect(Node):
    def __init__(self):
        super().__init__('sonar_polar_to_rect')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.cv_bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/pontus/sonar_0/image_debug',
            self.image_callback,
            qos_profile=qos_profile
        )

        self.point_cloud_pub = self.create_publisher(
            PointCloud2,
            '/pontus/sonar/rect',
            10
        )

    def image_callback(self, msg: Image) -> None:

        points = []
        bgr = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        shape = bgr.shape
        for r in range(0, shape[0]):
            for c in range(0, shape[1]):
                pixel = bgr[r][c]
                
                # average the channels
                value = sum(pixel) / 3.0

                if value > 10:
                    # convert to rectangular coords

                    sonar_res = 0.008
                    d = sonar_res * r

                    sonar_angle = math.pi/3.0
                    angle_step = 2 * sonar_angle / shape[1]
                    theta = (c * angle_step) - sonar_angle

                    x = d * math.cos(theta)
                    y = d * math.sin(theta)

                    point = np.array((x, y, 0.0, value / 255.0))
                    points.append(point)

        # Convert back to PointCloud2 and publish
        msg.header.frame_id = 'sonar_0'
        # point_msg = pc2.create_cloud_xyz32(msg.header, np.array(points))
        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='intensity', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
        ]

        point_msg = pc2.create_cloud(msg.header, fields, np.array(points))
        self.point_cloud_pub.publish(point_msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SonarPolarToRect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
