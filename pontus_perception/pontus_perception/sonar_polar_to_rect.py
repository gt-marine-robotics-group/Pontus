import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
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

        sonar_range = 15 # Needs to match what is configured in the driver in m
        sonar_angle = 2 * math.pi/3.0 # 120 degrees
        sonar_res = sonar_range / shape[0]

        # Loops vertically (rows) which corresponds to distance along beam
        for distance in range(0, shape[0]):
            # Loops horizontally (cols) which corresponds to beam angle
            for angle in range(0, shape[1]):
                pixel = bgr[r][c]

                # average the channels
                value = sum(pixel) / 3.0

                # Ignore very low intensity returns which are probably noise
                if value > 1:
                    # convert to rectangular coords

                    d = sonar_res * distance # convert distance in pixels to m

                    angle_step = sonar_angle / shape[1]
                    theta = (angle * angle_step) - sonar_angle

                    x = d * math.cos(theta)
                    y = d * math.sin(theta)

                    point = np.array((x, y, 0.0, value / 255.0))
                    points.append(point)

        # Convert back to PointCloud2 and publish
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
