import cv2
from cv_bridge import CvBridge
from typing import List, Optional
import numpy as np
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from message_filters import Subscriber, ApproximateTimeSynchronizer

from pontus_msgs.msg import YOLOResultArray, YOLOResult, YOLOResultArrayPose
from pontus_mapping.semantic_map_manager import SemanticObject

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

        yolo_sub = Subscriber(
            self,
            YOLOResultArray,
            '/pontus/camera_2/yolo_results',
        )
        image_rect_sub = Subscriber(
            self,
            Image,
            '/pontus/camera_2/image_rect_color',
        )
        self.yolo_result_array_distances_pub = self.create_publisher(
            YOLOResultArrayPose,
            '/pontus/camera_2/yolo_results_pose',
            10
        )
        self.ts = ApproximateTimeSynchronizer([yolo_sub, image_rect_sub], 10, 0.1)
        self.ts.registerCallback(self.callback)
        self.bridge = CvBridge()

    def callback(self, msg: YOLOResultArray, image_msg: Image) -> None:
        """
        Take yolo results and publish poses for each object.

        Args:
        ----
        msg (YOLOResultArray): yolo results
        image_msg (Image): image message

        Return:
        ------
        None

        """
        pub_msg = YOLOResultArrayPose()
        # TODO: Test this for other cylinder objects such as slalom and gate
        for yolo_result in msg.results:
            if yolo_result.class_id == SemanticObject.VerticalMarker.value:
                pose = self.get_pose(yolo_result, image_msg)
                pub_msg.results.append(yolo_result)
                self.get_logger().info(f"publishing: {pose}")
                pub_msg.distances.append(pose)
        self.yolo_result_array_distances_pub.publish(pub_msg)

    def get_line_center(self, lines: np.ndarray) -> np.ndarray:
        """
        Calculate the center of the lines.

        This will make clustering more accurcate.

        Args:
        ----
        lines (np.ndarray): lines from hough line transform

        Return:
        ------
        np.ndarray: array of center points

        """
        rho_values = lines[:, 0]
        theta_values = lines[:, 1]
        x_center = rho_values * np.cos(theta_values)
        y_center = rho_values * np.sin(theta_values)
        center_points = np.column_stack((x_center, y_center))
        return center_points

    def cluster_lines(self, lines: np.ndarray) -> np.ndarray:
        """
        Cluster the lines to remove duplicate detections.

        Args:
        ----
        lines (np.ndarray): array of lines from hough line transform

        Return:
        ------
        np.ndarray: clustered array of lines

        """
        points = self.get_line_center(lines)
        db = DBSCAN(eps=8, min_samples=1, metric='euclidean').fit(points)
        labels = db.labels_
        new_lines = []
        for label in set(labels):
            # Skip invalid cluster
            if label == -1:
                continue

            cluster_lines = lines[labels == label]
            avg_rho = np.mean(cluster_lines[:, 0])
            avg_theta = np.mean(cluster_lines[:, 1])
            new_lines.append((avg_rho, avg_theta))
        return np.array(new_lines)

    def angle_diff(self, angle1: float, angle2: float) -> float:
        """
        Calculate the difference between two angles.

        Args:
        ----
        angle1 (float): first angle
        angle2 (float): second angle

        Return:
        ------
        float: the difference in the angle

        """
        angle_diff = np.abs(angle1 - angle2)

        # Ensure the angle difference is within [-π, π]
        if angle_diff > np.pi:
            angle_diff = 2 * np.pi - angle_diff

        return angle_diff

    def get_size(self, lines: np.ndarray) -> float:
        """
        Return the size between two lines.

        Args:
        ----
        lines (np.ndarray): clustered lines of the object

        Return:
        ------
        float: the size of the object

        """
        if len(lines) != 2:
            self.get_logger().info(f"Skipping line detection, found {len(lines)} lines")
            return None

        # self.get_logger().info(f"Angle diff: {self.angle_diff(lines[0][1], lines[1][1])}")
        if self.angle_diff(lines[0][1], lines[1][1]) > 0.3:
            self.get_logger().info("Skipping line detection, lines not rougthly parralel")
            return None

        points = self.get_line_center(lines)
        distance = np.linalg.norm(points[0] - points[1])
        return distance

    def draw_lines(self, lines: np.ndarray, color_detection: np.ndarray) -> None:
        """
        Draw the clustered lines on an image for debugging.

        Args:
        ----
        lines (np.ndarray): the clustered lines
        color_detection (np.ndarray): the image to draw the lines on

        Return:
        ------
        None

        """
        for line in lines:
            rho, theta = line
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(color_detection, (x1, y1), (x2, y2), (0, 0, 255), 2)

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
        pose.position.x = -1.0
        image = self.bridge.imgmsg_to_cv2(current_image)

        # Extract bounding box from image
        x1, x2 = int(yolo_result.x1), int(yolo_result.x2)
        y1, y2 = int(yolo_result.y1), int(yolo_result.y2)
        detection_slice = image[max(y1 - 5, 0): y2 + 5, max(x1 - 5, 0): x2 + 5]

        # Preprocess image
        # blurred = cv2.bilateralFilter(detection_slice, d=9, sigmaColor=75, sigmaSpace=75)
        # Detect lines
        edges = cv2.Canny(detection_slice, threshold1=25, threshold2=200)
        lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=100)
        color_detection = detection_slice.copy()
        if lines is not None:
            lines = np.array([line[0] for line in lines])
            clustered_lines = self.cluster_lines(lines)
            size = self.get_size(clustered_lines)
            if size is None:
                return pose
            # TODO: Make a smart way to do this
            distance = 42 * 3 / size
            pose.position.x = distance
            self.get_logger().info(f"Distance: {distance}")
            self.draw_lines(lines, color_detection)

        cv2.imshow("detection_slice", detection_slice)
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
