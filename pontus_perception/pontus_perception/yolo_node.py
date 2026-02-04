# ------ Imports ------
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.timer import Timer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

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
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion
from vision_msgs.msg import Pose2D

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
                ('threshold', 0.45),
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
        self.cv_bridge: CvBridge = CvBridge()

        # ------ Subscribers / Puslishers ------
        self.img_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Only use compressed images if the raw image topic is not available
        self.image_sub_compressed = None
        if len(self.get_publishers_info_by_topic('input')) > 0:
            self.image_sub: Subscription = self.create_subscription(
                Image, 'input', self.image_callback, self.img_qos
            )
        else:
            self.image_sub_compressed = self.create_subscription(
                CompressedImage, 'input/compressed', self.image_callback_compressed, self.img_qos
            )

        self.detections_publisher: Publisher = self.create_publisher(
            Detection2DArray, 'results', 10
        )

        self.debug_image_pub: Publisher = self.create_publisher(
            Image, 'yolo_debug', self.img_qos
        )

        self.debug_image_compressed_pub: Publisher = self.create_publisher(
            CompressedImage, 'yolo_debug/compressed', self.img_qos
        )

        # Reenables the compressed image subscription if the raw image topic is not available
        self.topic_check_timer: Timer = self.create_timer(0.5, self.topic_check_callback)

    # ------ Helpers ------
    def _default_pose_with_covariance(self) -> PoseWithCovariance:
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
        if self.image_sub_compressed is not None:
            self.destroy_subscription(self.image_sub_compressed)
            self.image_sub_compressed = None

        try:
            image_bgr = self.cv_bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}')
            return
        self._run_yolo_and_publish(image_bgr, msg.header)

    def image_callback_compressed(self, msg: CompressedImage) -> None:
        try:
            image_bgr = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().warn(f'cv_bridge (compressed) failed: {e}')
            return
        self._run_yolo_and_publish(image_bgr, msg.header)

    def _run_yolo_and_publish(self, image_bgr: np.ndarray, src_header: Header) -> None:
        try:
            inference_results = self.model(image_bgr)[0]
        except Exception as e:
            self.get_logger().warn(f'YOLO inference failed: {e}')
            return

        detections_array = Detection2DArray()
        detections_array.header = _build_header(
            self, src_header, self.frame_id)

        xyxy = np.zeros((0, 4), dtype=float)
        confs = np.zeros((0,), dtype=float)
        clss = np.zeros((0,), dtype=int)
        if hasattr(inference_results, 'boxes') and inference_results.boxes is not None and len(inference_results.boxes) > 0:
            try:
                xyxy = inference_results.boxes.xyxy.detach().cpu().numpy()
                confs = inference_results.boxes.conf.detach().cpu().numpy()
                clss = inference_results.boxes.cls.detach().cpu().numpy().astype(int)
            except Exception as e:
                self.get_logger().warn(f'Failed to extract boxes: {e}')

        name_map: Dict[int, str] = getattr(
            inference_results, 'names', None) or getattr(self.model, 'names', {}) or {}
        kept = 0
        for (x1, y1, x2, y2), conf, cls_id in zip(xyxy, confs, clss):
            if float(conf) < self.threshold:
                continue

            width = float(max(0.0, x2 - x1))
            height = float(max(0.0, y2 - y1))
            cx = float(x1 + width / 2.0)
            cy = float(y1 + height / 2.0)

            bbox = BoundingBox2D()
            bbox.center.theta = 0.0
            bbox.center.position.x = cx
            bbox.center.position.y = cy
            bbox.size_x = width
            bbox.size_y = height

            label = name_map.get(int(cls_id), str(int(cls_id)))
            hypothesis = ObjectHypothesis(class_id=label, score=float(conf))
            hypothesis_with_pose = ObjectHypothesisWithPose(
                hypothesis=hypothesis,
                pose=self._default_pose_with_covariance()
            )

            det = Detection2D()
            det.header = detections_array.header
            det.bbox = bbox
            det.results = [hypothesis_with_pose]
            detections_array.detections.append(det)

            kept += 1

            # debug draw
            cv2.rectangle(image_bgr, (int(x1), int(y1)),
                          (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(image_bgr, f'{label}: {float(conf):.2f}', (int(x1), int(max(0, y1 - 8))),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        self.detections_publisher.publish(detections_array)
        try:
            self.debug_image_pub.publish(
                self.cv_bridge.cv2_to_imgmsg(image_bgr, encoding='bgr8'))
            self.debug_image_compressed_pub.publish(
                self.cv_bridge.cv2_to_compressed_imgmsg(image_bgr))
        except Exception as e:
            self.get_logger().warn(f'Failed to publish debug images: {e}')
        self.get_logger().debug(
            f'YOLO kept {kept}/{len(confs)} boxes (threshold={self.threshold:.2f})')

    def topic_check_callback(self):
        if self.image_sub_compressed is None and self.image_sub.get_publisher_count() == 0:
            self.image_sub_compressed = self.create_subscription(
                CompressedImage, 'input/compressed', self.image_callback_compressed, self.img_qos
            )

# ------ Main ------
def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: YOLONode = YOLONode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
