#!/usr/bin/env python3
"""Multi-camera YOLO detection node with TensorRT acceleration support.

Subscribes to image topics for 1..N cameras (driven by the ``cameras``
list in topics.yaml), runs batched inference across all cameras that
have new frames, and publishes Detection2DArray messages per camera.

Topic names come from topics.yaml via TopicConfig.  Compressed-image
fallback is automatic: the node tries raw ``Image`` first and switches
to ``CompressedImage`` if no raw publisher is found.

The node auto-detects the TensorRT engine's max batch size and chunks
inference accordingly — a batch=1 engine runs per-image, while a
batch=N engine batches up to N images in a single call.
"""

import os
import time
import traceback
from typing import Dict, List, Optional, Union

import torch
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO

from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from ament_index_python.packages import get_package_share_directory

from pontus_bringup.topic_config import TopicConfig


# ------ Helpers ------

def _build_header(
    node: Node, source_header: Header, fallback_frame_id: str
) -> Header:
    """Build a validated header, falling back to node clock if stamp is zero.

    Gazebo and bag-replay can produce messages with a zero timestamp.
    Using the node clock in that case keeps the TF pipeline happy.
    """
    header = Header()
    header.frame_id = source_header.frame_id or fallback_frame_id
    if source_header.stamp.sec != 0 or source_header.stamp.nanosec != 0:
        header.stamp = source_header.stamp
    else:
        header.stamp = node.get_clock().now().to_msg()
    return header


# ------ Per-Camera State ------

class _CamState:
    """Mutable bookkeeping for a single camera in the multi-cam node."""

    __slots__ = (
        'name', 'image_topic', 'optical_frame',
        'det_pub', 'overlay_pub',
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

        self.raw_sub: Optional[Subscription] = None
        self.compressed_sub: Optional[Subscription] = None

        self.latest_msg: Optional[Union[Image, CompressedImage]] = None
        self.last_proc_stamp: Optional[tuple] = None
        self.is_compressed: bool = False

        self.rx_count: int = 0
        self.last_rx_wall: Optional[float] = None
        self.last_det_count: Optional[int] = None


# ------ Node ------

class MultiCamYoloNode(Node):
    """Batched YOLO inference across 1..N cameras."""

    def __init__(self) -> None:
        super().__init__("multi_cam_yolo")

        # ------ Topics from config ------
        self.topics = TopicConfig(self, [
            'cameras',
            'image_topic_template',
            'detections_topic_template',
            'camera_frame_template',
            'overlay_topic_template',
        ])

        # ------ Model load ------
        self.declare_parameter('model_path', '')
        self.declare_parameter('threshold', 0.6)
        self.declare_parameter('auv', 'auv')
        auv: str = str(self.get_parameter('auv').value)
        model_path_param: str = str(self.get_parameter('model_path').value)
        self.threshold: float = float(self.get_parameter('threshold').value)

        resolved_model = self._resolve_model_path(model_path_param, auv)
        have_cuda = torch.cuda.is_available()
        self.device = 0 if have_cuda else "cpu"

        self.class_names: Dict[int, str] = {
            0: 'duck', 1: 'gate_side', 2: 'path_marker', 3: 'red_slalom',
            4: 'reef_shark', 5: 'saw_shark', 6: 'vertical pole', 7: 'white_slalom',
        }

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

        # ------ Batch size detection ------
        self.max_batch_size: int = self._get_max_batch_size(resolved_model)
        self.get_logger().info(
            f'Max batch size: {self.max_batch_size} '
            f'(-1 = unlimited)')

        # ------ CvBridge ------
        self.bridge: CvBridge = CvBridge()

        # ------ QoS ------
        self.img_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ------ Per-camera setup ------
        self.cam_states: Dict[str, _CamState] = {}
        for cam_name in self.topics.cameras:
            self._setup_camera(cam_name)

        # ------ Batching state ------
        self.processing: bool = False

        # ------ Debug counters ------
        self.tick_calls: int = 0
        self.proc_calls: int = 0
        self.last_infer_ms: Optional[float] = None

        # ------ Timers ------
        self.create_timer(0.1, self._tick)
        self.create_timer(1.0, self._debug_tick)
        self.create_timer(0.5, self._topic_check)

        cam_names = [c.name for c in self.cam_states.values()]
        self.get_logger().info(f"MultiCam YOLO ready — cameras: {cam_names}")

    # ------ Model Resolution ------

    def _resolve_model_path(self, param_value: str, auv: str) -> str:
        """Return model path with priority: engine -> param override -> .pt fallback.

        TensorRT engines are preferred for inference speed on the Jetson.
        A user-provided model_path param is used if no engine exists.
        Falls back to the default .pt model as a last resort.
        """
        pkg_share = get_package_share_directory('pontus_perception')
        engine_path = os.path.join(pkg_share, 'yolo', auv, 'model.engine')
        pt_path = os.path.join(pkg_share, 'yolo', auv, 'model.pt')

        if os.path.exists(engine_path):
            return engine_path

        if param_value and os.path.exists(param_value):
            return param_value

        self.get_logger().warn(
            f'Engine not found. Falling back to {pt_path}')
        return pt_path

    # ------ Batch Size Detection ------

    def _get_max_batch_size(self, model_path: str) -> int:
        """Probe the model for its maximum supported batch size.

        TensorRT engines have a fixed batch dimension baked in at export
        time.  PyTorch models support dynamic batching (returns -1).
        """
        if not model_path.endswith('.engine'):
            return -1  # .pt models accept any batch size

        try:
            import tensorrt as trt
            logger = trt.Logger(trt.Logger.WARNING)
            with open(model_path, 'rb') as f:
                runtime = trt.Runtime(logger)
                engine = runtime.deserialize_cuda_engine(f.read())
            input_shape = engine.get_binding_shape(0)
            batch_dim = int(input_shape[0])
            self.get_logger().info(
                f'TRT engine input shape: {list(input_shape)}')
            return batch_dim if batch_dim > 0 else -1
        except Exception as e:
            self.get_logger().warn(
                f'Could not probe engine batch size: {e}. Defaulting to 1.')
            return 1

    # ------ Per-Camera Setup ------

    def _setup_camera(self, cam_name: str) -> None:
        """Create subscribers, publishers, and state for one camera."""
        image_topic = self.topics.image_topic_template.replace(
            '{camera}', cam_name)
        det_topic = self.topics.detections_topic_template.replace(
            '{camera}', cam_name)
        optical_frame = self.topics.camera_frame_template.replace(
            '{camera}', cam_name)
        overlay_topic = self.topics.overlay_topic_template.replace(
            '{camera}', cam_name)

        cs = _CamState(cam_name, image_topic, optical_frame)

        # Detection publisher
        cs.det_pub = self.create_publisher(Detection2DArray, det_topic, 10)

        # Overlay publisher (debug annotated image)
        cs.overlay_pub = self.create_publisher(
            Image, overlay_topic, self.img_qos)

        # Start with raw image subscription
        cs.raw_sub = self.create_subscription(
            Image, image_topic,
            self._make_image_cb(cs, compressed=False),
            self.img_qos,
        )

        self.cam_states[cam_name] = cs
        self.get_logger().info(
            f"  Camera '{cam_name}': image={image_topic}, "
            f"det={det_topic}, overlay={overlay_topic}, frame={optical_frame}")

    def _make_image_cb(
        self, cs: _CamState, compressed: bool
    ):
        """Factory for per-camera image callbacks."""
        def callback(msg: Union[Image, CompressedImage]) -> None:
            # If we get a raw frame, we no longer need the compressed sub
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
        """Re-subscribe to compressed topics for cameras with no raw publisher."""
        for cs in self.cam_states.values():
            if cs.compressed_sub is not None:
                continue
            # If no raw frames received yet, try compressed
            if cs.latest_msg is None:
                compressed_topic = cs.image_topic + "/compressed"
                cs.compressed_sub = self.create_subscription(
                    CompressedImage, compressed_topic,
                    self._make_image_cb(cs, compressed=True),
                    self.img_qos,
                )

    # ------ Decoding ------

    def _decode(
        self, msg: Union[Image, CompressedImage], compressed: bool
    ) -> Optional[np.ndarray]:
        """Decode raw or compressed image to BGR numpy array.

        Returns None if the image is empty.  Ensures the output is
        contiguous in memory to prevent TensorRT segfaults.
        """
        if compressed:
            img = self.bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding="bgr8")
        else:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if img is None or img.size == 0:
            return None
        return np.ascontiguousarray(img).copy()

    # ------ Detection Conversion ------

    def _ultra_to_det2d_array(
        self, ultralytics_result, source_header: Header, optical_frame: str
    ) -> Detection2DArray:
        """Convert an Ultralytics result to a Detection2DArray."""
        out = Detection2DArray()
        out.header = _build_header(self, source_header, optical_frame)
        out.header.frame_id = optical_frame

        boxes = ultralytics_result.boxes
        if boxes is None or len(boxes) == 0:
            return out

        xyxy = boxes.xyxy.cpu().numpy()
        conf = boxes.conf.cpu().numpy()
        cls = boxes.cls.cpu().numpy().astype(int)

        for (x1, y1, x2, y2), score, cid in zip(xyxy, conf, cls):
            det = Detection2D()
            det.header = _build_header(self, source_header, optical_frame)
            det.header.frame_id = optical_frame

            det.bbox.center.position.x = float(0.5 * (x1 + x2))
            det.bbox.center.position.y = float(0.5 * (y1 + y2))
            det.bbox.center.theta = 0.0
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            hyp = ObjectHypothesisWithPose()
            if cid in self.class_names:
                hyp.hypothesis.class_id = self.class_names[cid]
                det.id = self.class_names[cid]
            else:
                hyp.hypothesis.class_id = str(cid)
            hyp.hypothesis.score = float(score)
            det.results.append(hyp)

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
                self.get_logger().warn(
                    f"[{cs.name}] decode failed: {e}")

        if not batch_imgs:
            return

        self.processing = True
        self.proc_calls += 1

        try:
            t0 = time.time()

            # Chunk images by max_batch_size.  -1 means unlimited (PyTorch).
            if self.max_batch_size == -1:
                chunks = [batch_imgs]
            else:
                chunks = [
                    batch_imgs[i:i + self.max_batch_size]
                    for i in range(0, len(batch_imgs), self.max_batch_size)
                ]

            results: list = []
            for chunk in chunks:
                res = self.model(
                    chunk, device=self.device,
                    verbose=False, conf=self.threshold, half=True)
                results.extend(res)

            self.last_infer_ms = (time.time() - t0) * 1000.0

            for cs, result in zip(batch_cams, results):
                # Publish annotated overlay only if someone is listening
                if cs.overlay_pub.get_subscription_count() > 0:
                    ann = result.plot()
                    overlay_msg = self.bridge.cv2_to_imgmsg(
                        ann, encoding="bgr8")
                    overlay_msg.header = cs.latest_msg.header
                    cs.overlay_pub.publish(overlay_msg)

                # Publish detections
                det_array = self._ultra_to_det2d_array(
                    result, cs.latest_msg.header, cs.optical_frame)
                cs.last_det_count = len(det_array.detections)
                cs.det_pub.publish(det_array)

                # Mark as processed
                cs.last_proc_stamp = (
                    cs.latest_msg.header.stamp.sec,
                    cs.latest_msg.header.stamp.nanosec,
                )

        except Exception as e:
            self.get_logger().error(f"Inference/publish exception: {e}")
            self.get_logger().error(traceback.format_exc())
        finally:
            self.processing = False

    # ------ Debug ------

    def _debug_tick(self) -> None:
        """1 Hz status log."""
        now = time.time()
        parts: List[str] = []
        for cs in self.cam_states.values():
            age = (f"{now - cs.last_rx_wall:.1f}s"
                   if cs.last_rx_wall else "None")
            src = "compressed" if cs.is_compressed else "raw"
            parts.append(
                f"{cs.name}(rx={cs.rx_count}, age={age}, "
                f"dets={cs.last_det_count}, src={src})")

        self.get_logger().info(
            f"STATUS: {' | '.join(parts)} | "
            f"ticks={self.tick_calls} procs={self.proc_calls} "
            f"infer_ms={self.last_infer_ms}")


# ------ Main ------

def main() -> None:
    rclpy.init()
    node = MultiCamYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
