import rclpy
from rclpy.node import Node
from pontus_msgs.msg import YOLOResultArray, YOLOResult
from pontus_mapping.semantic_map_manager import SemanticObject
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import cv2
from cv_bridge import CvBridge
from typing import List, Optional
import numpy as np

"""
This node can be used to detect the position of cylinder shaped objects.

I would imagine this would work for sub 2025:
- slalom
- vertical marker
- gate

NOTE: This should not be used for actual pose detection, but will give a rough estimate of how
far the object is from the current sub.
"""


class CylinderShapeDetection(Node):
    def __init__(self):
        super().__init__('cylinder_shape_detection')

        self.create_subscription(
            YOLOResultArray,
            '/pontus/camera_2/yolo_results',
            self.yolo_callback,
            10
        )
        self.create_subscription(
            Image,
            '/pontus/camera_2/image_rect_color',
            self.image_callback,
            10
        )
        self.current_image = None
        self.bridge = CvBridge()

    def image_callback(self, msg: Image) -> None:
        """
        Handle image callback.

        Args:
        ----
        msg (Image): image from topic

        Return:
        ------
        None

        """
        self.current_image = msg

    def yolo_callback(self, msg: YOLOResultArray) -> None:
        """
        Take yolo results and publish poses for each object.

        Args:
        ----
        msg (YOLOResultArray): yolo results

        Return:
        ------
        None

        """
        if not self.current_image:
            self.get_logger().warn('Have not received colored image, skipping')
            return
        # TODO: Test this for other cylinder objects such as slalom and gate
        for yolo_result in msg.results:
            if yolo_result.class_id == SemanticObject.VerticalMarker.value:
                self.get_pose(yolo_result, self.current_image)
    
    def get_pose(self, yolo_result: YOLOResult, current_image: Image) -> Pose:
        """
        Return the pose of a yolo detection based on its size.

        Args:
        ----
        yolo_result (YOLOResult): yolo detection we want to find the pose of
        current_image (Image): our current image

        Return:
        ------
        Pose: the pose of the object

        """
        pose = Pose()
        image = self.bridge.imgmsg_to_cv2(current_image)
        x1, x2 = int(yolo_result.x1), int(yolo_result.x2)
        y1, y2 = int(yolo_result.y1), int(yolo_result.y2)
        detection_slice = image[y1:y2, x1:x2]
        blurred = cv2.GaussianBlur(detection_slice, (11, 11), 0)
        edges = cv2.Canny(blurred, threshold1=50, threshold2=200)
        lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=150)
        color_detection = detection_slice.copy()
        if lines is not None:
            for line in lines:
                rho, theta = line[0]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a* rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                cv2.line(color_detection, (x1, y1), (x2, y2), (0, 0, 255), 2)

        cv2.imshow("detection_slice", blurred)
        cv2.imshow("detection_slice_lines", color_detection)
        cv2.imshow('edges', edges)
        cv2.waitKey(1)
        return pose


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = CylinderShapeDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()