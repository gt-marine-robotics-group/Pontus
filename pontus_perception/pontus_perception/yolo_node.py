#!/usr/bin/env python3
"""Multi-camera YOLO detection node with TensorRT acceleration support.

Subscribes to image topics for left and right cameras, runs batched
inference on a 10 Hz timer (skipping duplicate frames), and publishes
Detection2DArray messages plus debug overlay images per camera.

Camera base topics are declared as ROS parameters so they can be
changed from the launch file or CLI without editing this file:

    ros2 run pontus_perception yolo \
        --ros-args \
        -p left_camera_topic:=/pontus/camera_left \
        -p right_camera_topic:=/pontus/camera_right

Ported from gt-bbx MultiCamYoloNode with Pontus TensorRT hardening.
"""

from __future__ import annotations

import os
import time
import traceback
from typing import Dict, List, Optional, Union

import cv2
import numpy as np
import torch
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.timer import Timer

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO

from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    BoundingBox2D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion
from ament_index_python.packages import get_package_share_directory


# ------ Helpers ------

def _build_header(
    node: Node, source_header: Header, fallback_frame_id: str
) -> Header:
    """Build a validated header, falling back to node clock if stamp is zero."""
    header = Header()
    header.frame_id = source_header.frame_id or fallback_frame_id
    if source_header.stamp.sec != 0 or source_header.stamp.nanosec != 0:
        header.stamp = source_header.stamp
    else:
        header.stamp = node.get_clock().now().to_msg()
    return header


def _default_pose_with_covariance() -> PoseWithCovariance:
    """PoseWithCovariance with identity orientation and zero covariance."""
    pwc = PoseWithCovariance()
    pwc.pose = Pose(
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    return pwc


# ------ Per-Camera State ------

class _CamState:
    """Mutable bookkeeping for a single camera."""

    __slots__ = (
        'name', 'image_topic', 'optical_frame',
        'det_pub', 'overlay_pub', 'overlay_compressed_pub',
        'raw_sub', 'compressed_sub',
        'latest_msg', 'last_proc_stamp',
        'rx_count', 'last_rx_wall', 'last_det_count',
        'is_compressed',
    )

    def __init__(self, name: str, image_topic: str, optical_frame: str) -> None:
        self.name: str = name
        self.image_topic: str = image_topic
        self.optical_frame: str = optical_frame

        self.det_pub: Optional[Publisher] = None
        self.overlay_pub: Optional[Publisher] = None
        self.overlay_compressed_pub: Optional[Publisher] = None

        self.raw_sub: Optional[Subscription] = None
        self.compressed_sub: Optional[Subscription] = None

        self.latest_msg: Optional[Union[Image, CompressedImage]] = None
        self.last_proc_stamp: Optional[tuple] = None
        self.is_compressed: bool = False

        self.rx_count: int = 0
        self.last_rx_wall: Optional[float] = None
        self.last_det_count: Optional[int] = None


# ------ Node ------

class YOLONode(Node):
    """Batched YOLO inference across left/right cameras."""

    def __init__(self) -> None:
        super().__init__('perception_YOLO')

        # ------ Parameters ------
        self.declare_parameters(
            namespace='',
            parameters=[
                ('auv', 'auv'),
                ('threshold', 0.45),
                ('model_path', ''),
                ('frame_id', ''),
                # Camera base topics — change these from CLI or launch file
                ('left_camera_topic', '/pontus/camera_left'),
                ('right_camera_topic', '/pontus/camera_right'),
            ],
        )

        auv: str = str(self.get_parameter('auv').value)
        self.threshold: float = float(self.get_parameter('threshold').value)
        model_path_param: str = str(self.get_parameter('model_path').value)
        self.frame_id: str = str(self.get_parameter('frame_id').value)
        left_topic: str = str(self.get_parameter('left_camera_topic').value)
        right_topic: str = str(self.get_parameter('right_camera_topic').value)

        # ------ Model resolution (engine -> .pt fallback) ------
        resolved_model = self._resolve_model_path(model_path_param, auv)

        # ------ Class names (TRT engines can strip metadata) ------
        self.class_names: Dict[int, str] = {
            0: 'duck', 1: 'gate_side', 2: 'path_marker', 3: 'red_slalom',
            4: 'reef_shark', 5: 'saw_shark', 6: 'vertical pole', 7: 'white_slalom',
        }

        # ------ Torch / YOLO ------
        self.device: str = 'cuda' if torch.cuda.is_available() else 'cpu'

        if resolved_model.endswith('.engine'):
            self.model = YOLO(resolved_model, task='detect')
            self.get_logger().info(f'RUNNING TENSORRT: {resolved_model}')

            def inject_metadata(predictor: object) -> None:
                if hasattr(predictor, 'model'):
                    predictor.model.names = self.class_names  # type: ignore[attr-defined]
            self.model.add_callback('on_predict_start', inject_metadata)
        else:
            self.model = YOLO(resolved_model).to(self.device)
            self.class_names = getattr(self.model, 'names', self.class_names)
            self.get_logger().info(f'RUNNING PYTORCH: {resolved_model}')

        # ------ CvBridge ------
        self.bridge: CvBridge = CvBridge()

        # ------ QoS ------
        self.img_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ------ Per-camera setup ------
        # Each camera gets: image_raw sub, yolo_results pub, yolo_debug pub
        self.cam_states: Dict[str, _CamState] = {}
        self._setup_camera('left', left_topic)
        self._setup_camera('right', right_topic)

        # ------ Batching state ------
        self.processing: bool = False

        # ------ Debug counters ------
        self.tick_calls: int = 0
        self.proc_calls: int = 0
        self.last_infer_ms: Optional[float] = None

        # ------ Timers ------
        self.create_timer(0.1, self._tick)         # 10 Hz inference
        self.create_timer(1.0, self._debug_tick)   # 1 Hz status log
        self.create_timer(0.5, self._topic_check)  # compressed fallback

        cam_names = list(self.cam_states.keys())
        self.get_logger().info(f'YOLO ready — cameras: {cam_names}')

    # ------ Model Resolution ------

    def _resolve_model_path(self, param_value: str, auv: str) -> str:
        """Return model path: param override -> .engine -> .pt fallback."""
        pkg_share = get_package_share_directory('pontus_perception')
        engine_path = os.path.join(pkg_share, 'yolo', auv, 'model.engine')
        pt_path = os.path.join(pkg_share, 'yolo', auv, 'model.pt')

        if param_value and os.path.exists(param_value):
            return param_value
        elif os.path.exists(engine_path):
            return engine_path
        else:
            self.get_logger().warn(
                f'Engine not found at {engine_path}. Falling back to .pt')
            return pt_path

    # ------ Per-Camera Setup ------

    def _setup_camera(self, name: str, base_topic: str) -> None:
        """Create subscribers, publishers, and state for one camera.

        Given a base_topic like ``/pontus/camera_left``, creates:
          - subscriber: ``/pontus/camera_left/image_raw``
          - detections: ``/pontus/camera_left/yolo_results``
          - debug:      ``/pontus/camera_left/yolo_debug``
        """
        image_topic = f'{base_topic}/image_raw'
        det_topic = f'{base_topic}/yolo_results'
        debug_topic = f'{base_topic}/yolo_debug'

        cs = _CamState(name, image_topic, self.frame_id)

        cs.det_pub = self.create_publisher(Detection2DArray, det_topic, 10)
        cs.overlay_pub = self.create_publisher(
            Image, debug_topic, self.img_qos)
        cs.overlay_compressed_pub = self.create_publisher(
            CompressedImage, f'{debug_topic}/compressed', self.img_qos)

        cs.raw_sub = self.create_subscription(
            Image, image_topic,
            self._make_image_cb(cs, compressed=False),
            self.img_qos,
        )

        self.cam_states[name] = cs
        self.get_logger().info(
            f"  Camera '{name}': image={image_topic}, det={det_topic}")

    def _make_image_cb(self, cs: _CamState, compressed: bool):
        """Factory for per-camera image callbacks."""
        def callback(msg: Union[Image, CompressedImage]) -> None:
            if not compressed and cs.compressed_sub is not None:
                self.destroy_subscription(cs.compressed_sub)
                cs.compressed_sub = None
            cs.latest_msg = msg
            cs.is_compressed = compressed
            cs.rx_count += 1
            cs.last_rx_wall = time.time()
        return callback

    # ------ Compressed Fallback ------

    def _topic_check(self) -> None:
        """Re-subscribe to compressed for cameras with no raw frames."""
        for cs in self.cam_states.values():
            if cs.compressed_sub is not None:
                continue
            if cs.latest_msg is None:
                compressed_topic = cs.image_topic + '/compressed'
                cs.compressed_sub = self.create_subscription(
                    CompressedImage, compressed_topic,
                    self._make_image_cb(cs, compressed=True),
                    self.img_qos,
                )

    # ------ Decoding ------

    def _decode(
        self, msg: Union[Image, CompressedImage], compressed: bool
    ) -> Optional[np.ndarray]:
        """Decode raw or compressed image to BGR numpy array."""
        if compressed:
            img = self.bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        else:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if img is None or img.size == 0:
            return None
        # Contiguous memory guard — prevents TensorRT segfaults
        return np.ascontiguousarray(img).copy()

    # ------ Detection Conversion ------

    def _result_to_det2d_array(
        self, inference_result, source_header: Header, optical_frame: str
    ) -> Detection2DArray:
        """Convert an Ultralytics result to a Detection2DArray."""
        out = Detection2DArray()
        out.header = _build_header(self, source_header, optical_frame)

        if (inference_result is None
                or not hasattr(inference_result, 'boxes')
                or inference_result.boxes is None
                or len(inference_result.boxes) == 0):
            return out

        try:
            xyxy = inference_result.boxes.xyxy.detach().cpu().numpy()
            confs = inference_result.boxes.conf.detach().cpu().numpy()
            clss = inference_result.boxes.cls.detach().cpu().numpy().astype(int)
        except Exception as e:
            self.get_logger().warn(f'Failed to extract boxes: {e}')
            return out

        name_map = getattr(inference_result, 'names', None) or self.class_names

        for (x1, y1, x2, y2), score, cid in zip(xyxy, confs, clss):
            if float(score) < self.threshold:
                continue

            width = float(max(0.0, x2 - x1))
            height = float(max(0.0, y2 - y1))

            bbox = BoundingBox2D()
            bbox.center.position.x = float(x1 + width / 2.0)
            bbox.center.position.y = float(y1 + height / 2.0)
            bbox.center.theta = 0.0
            bbox.size_x = width
            bbox.size_y = height

            label = name_map.get(int(cid), str(int(cid)))
            hyp = ObjectHypothesisWithPose(
                hypothesis=ObjectHypothesis(
                    class_id=label, score=float(score)),
                pose=_default_pose_with_covariance(),
            )

            det = Detection2D()
            det.header = out.header
            det.bbox = bbox
            det.results = [hyp]
            out.detections.append(det)

        return out

    # ------ Tick (Batched Inference) ------

    def _tick(self) -> None:
        """10 Hz timer: collect new frames from all cameras, batch inference."""
        self.tick_calls += 1

        if self.processing:
            return

        # Collect cameras with new frames
        batch_cams: List[_CamState] = []
        batch_imgs: List[np.ndarray] = []

        for cs in self.cam_states.values():
            if cs.latest_msg is None:
                continue
            stamp = (cs.latest_msg.header.stamp.sec,
                     cs.latest_msg.header.stamp.nanosec)
            if stamp == cs.last_proc_stamp:
                continue
            try:
                img = self._decode(cs.latest_msg, cs.is_compressed)
                if img is None:
                    continue
                batch_cams.append(cs)
                batch_imgs.append(img)
            except Exception as e:
                self.get_logger().warn(f'[{cs.name}] decode failed: {e}')

        if not batch_imgs:
            return

        self.processing = True
        self.proc_calls += 1

        try:
            t0 = time.time()
            results = self.model(
                batch_imgs, device=self.device,
                verbose=False, conf=self.threshold, half=True)
            self.last_infer_ms = (time.time() - t0) * 1000.0

            for cs, result in zip(batch_cams, results):
                # Publish detections
                det_array = self._result_to_det2d_array(
                    result, cs.latest_msg.header, cs.optical_frame)
                cs.last_det_count = len(det_array.detections)
                cs.det_pub.publish(det_array)

                # Publish annotated overlay only if someone is listening
                raw_subs = cs.overlay_pub.get_subscription_count()
                comp_subs = cs.overlay_compressed_pub.get_subscription_count()
                if raw_subs > 0 or comp_subs > 0:
                    ann = result.plot()
                    if raw_subs > 0:
                        overlay_msg = self.bridge.cv2_to_imgmsg(
                            ann, encoding='bgr8')
                        overlay_msg.header = cs.latest_msg.header
                        cs.overlay_pub.publish(overlay_msg)
                    if comp_subs > 0:
                        comp_msg = self.bridge.cv2_to_compressed_imgmsg(ann)
                        comp_msg.header = cs.latest_msg.header
                        cs.overlay_compressed_pub.publish(comp_msg)

                # Mark as processed
                cs.last_proc_stamp = (
                    cs.latest_msg.header.stamp.sec,
                    cs.latest_msg.header.stamp.nanosec,
                )

        except Exception as e:
            self.get_logger().error(f'Inference/publish exception: {e}')
            self.get_logger().error(traceback.format_exc())
        finally:
            self.processing = False

    # ------ Debug ------

    def _debug_tick(self) -> None:
        """1 Hz status log."""
        now = time.time()
        parts: List[str] = []
        for cs in self.cam_states.values():
            age = (f'{now - cs.last_rx_wall:.1f}s'
                   if cs.last_rx_wall else 'None')
            src = 'compressed' if cs.is_compressed else 'raw'
            parts.append(
                f'{cs.name}(rx={cs.rx_count}, age={age}, '
                f'dets={cs.last_det_count}, src={src})')

        self.get_logger().info(
            f"STATUS: {' | '.join(parts)} | "
            f'ticks={self.tick_calls} procs={self.proc_calls} '
            f'infer_ms={self.last_infer_ms}')


# ------ Main ------

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = YOLONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
