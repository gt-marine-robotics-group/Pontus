# ------ Imports ------
from __future__ import annotations

import rclpy
from rclpy.node import Node, Subscription, Publisher
from rclpy.clock import Clock

from typing import Optional, List, Dict

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage

from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    BoundingBox2D,
    ObjectHypothesis,
    ObjectHypothesisWithPose
)
from geometry_msgs.msg import Pose2D, PoseWithCovariance, Pose, Point, Quaternion

from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import torch
import numpy as np
import cv2

# ------ Helpers ------


def _build_header(node: Node, source_header: Header, fallback_frame_id: str) -> Header:
    """Return a header using the input header if valid, otherwise use the node clock."""
    header = Header()
    header.frame_id = source_header.frame_id or fallback_frame_id
    if source_header.stamp.sec != 0 or source_header.stamp.nanosec != 0:
        header.stamp = source_header.stamp
    else:
        header.stamp = node.get_clock().now().to_msg()
    return header


# ------ YOLO Node ------
class YOLONode(Node):
    """
    Node that subscribes to an image, runs YOLO, and publishes detections as
    vision_msgs/Detection2DArray plus a debug image.
    """

    def __init__(self) -> None:

        super().__init__('perception_YOLO')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('auv', 'auv'),
                ('threshold', '0.45'),
                ('model_path', ''),
                ('frame_id', '')
            ]
        )

        auv = str(self.get_parameter('auv').value)
        threshold = float(self.get_parameter('threshold').value)
        model_path_param = str(self.get_parameter('model_path').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        if model_path_param:
            model_path = model_path_param
        else:
            pkg_share = get_package_share_directory('pontus_perception')
            model_path = f'{pkg_share}/yolo/{auv}/model.pt'

        self.threshold = threshold

        # ------ Torch / YOLO ------
        self.device: str = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = YOLO(model_path).to(self.device)
        self.get_logger().info(
            f'YOLO loaded on device="{self.device}" from "{model_path}"')

        # ------ CVBridge ------
        self.bridge: CvBridge = CvBridge()

        # ------ Subscribers / Puslishers ------
        self.image_sub: Subscription = self.create_subscription(
            Image, 'input', self.image_callback, 10
        )

        self.detections_publisher: Publisher = self.create_publisher(
            Detection2DArray, 'results', 10
        )

        self.debug_image_pub: Publisher = self.create_publisher(
            Image, 'yolo_debug', 10
        )

        self.debug_image_compressed_pub: Publisher = self.create_publisher(
            CompressedImage, 'yolo_debug/compressed', 10
        )

    # ------ Helpers ------
    @staticmethod
    def _default_pose_with_covariance() -> PoseWithCovariance:
        """
        Create a PoseWithCovariance with identity orientation and zero covariance.
        """
        pose_with_covariance: PoseWithCovariance = PoseWithCovariance()
        pose_with_covariance.pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        # Leave covariance as zeros to indicate "unknown"
        return pose_with_covariance

    # ------ Callback ------
    def image_callback(self, msg: Image) -> None:
        """
        Take in an image and run YOLO on the image.

        Args:
        ----
        msg (Image): the image we want to run YOLO on

        Return:
        ------
        None

        """

        image_bgr: np.ndarray = self.cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')

        inference_results = self.model(image_bgr)[0]

        detections_array: Detection2DArray = Detection2DArray()
        detections_array.header = _build_header(
            self, msg.header, self.frame_id)

        # Extract boxes, confidences, classes
        if inference_results.boxes is not None and len(inference_results.boxes) > 0:
            xyxy: np.ndarray = inference_results.boxes.xyxy.detach().cpu().numpy()  # (N, 4)
            confs: np.ndarray = inference_results.boxes.conf.detach().cpu().numpy()  # (N,)
            clss: np.ndarray = inference_results.boxes.cls.detach(
            ).cpu().numpy().astype(int)  # (N,)
        else:
            xyxy = np.zeros((0, 4), dtype=float)
            confs = np.zeros((0,), dtype=float)
            clss = np.zeros((0,), dtype=int)

        # Class name mapping (prefer per-result mapping)
        name_map: Dict[int, str] = getattr(inference_results, 'names', None) \
            or getattr(self.model, 'names', {}) \
            or {}

        # Build messages and draw debug
        for (x1, y1, x2, y2), conf, cls_id in zip(xyxy, confs, clss):
            if float(conf) < self.threshold:
                continue

            width: float = float(max(0.0, x2 - x1))
            height: float = float(max(0.0, y2 - y1))
            cx: float = float(x1 + width / 2.0)
            cy: float = float(y1 + height / 2.0)

            bbox: BoundingBox2D = BoundingBox2D(
                center=Pose2D(x=cx, y=cy, theta=0.0),
                size_x=width,
                size_y=height,
            )

            label: str = name_map.get(int(cls_id), str(int(cls_id)))
            hypothesis: ObjectHypothesis = ObjectHypothesis(
                class_id=label, score=float(conf))
            hypothesis_with_pose: ObjectHypothesisWithPose = ObjectHypothesisWithPose(
                hypothesis=hypothesis,
                pose=self.default_pose_with_covariance(),
            )

            detection: Detection2D = Detection2D()
            detection.header = detections_array.header
            detection.bbox = bbox
            detection.results = [hypothesis_with_pose]
            detections_array.detections.append(detection)

            # Debug draw
            cv2.rectangle(image_bgr, (int(x1), int(y1)),
                          (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(
                image_bgr,
                f'{label}: {float(conf):.2f}',
                (int(x1), int(max(0, y1 - 8))),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        # Publish results
        self.detections_publisher.publish(detections_array)

        # Publish debug images
        self.debug_image_pub.publish(
            self.bridge.cv2_to_imgmsg(image_bgr, encoding='bgr8'))
        self.debug_image_compressed_pub.publish(
            self.bridge.cv2_to_compressed_imgmsg(image_bgr)
        )


# ------ Main ------
def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: YOLONode = YOLONode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
