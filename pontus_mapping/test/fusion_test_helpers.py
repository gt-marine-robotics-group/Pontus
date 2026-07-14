"""Message and geometry builders shared by fusion unit/integration tests."""

from __future__ import annotations

from math import sqrt
from types import SimpleNamespace
from typing import Iterable

import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from vision_msgs.msg import Detection2D, Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose


def make_stamp(seconds: float) -> TimeMsg:
    whole_seconds = int(seconds)
    nanoseconds = int(round((seconds - whole_seconds) * 1.0e9))
    if nanoseconds >= 1_000_000_000:
        whole_seconds += 1
        nanoseconds -= 1_000_000_000
    stamp = TimeMsg()
    stamp.sec = whole_seconds
    stamp.nanosec = nanoseconds
    return stamp


def stamp_namespace(seconds: float):
    stamp = make_stamp(seconds)
    return SimpleNamespace(sec=stamp.sec, nanosec=stamp.nanosec)


def make_class_config(
    module,
    semantic_id: int = 1,
    *,
    bearing_margin_deg: float = 5.0,
    max_track_range_m: float = 20.0,
    label_evidence_threshold: float = 2.0,
    promotion_confidence: float = 0.65,
    promotion_margin: float = 0.5,
    stereo_max_error_m: float | None = 1.0,
    stereo_min_parallax_deg: float = 3.0,
):
    return module.ClassFusionConfig(
        semantic_id=semantic_id,
        min_detector_confidence=0.40,
        min_bbox_width_px=1.0,
        min_bbox_height_px=1.0,
        bearing_margin_deg=bearing_margin_deg,
        max_track_range_m=max_track_range_m,
        label_evidence_threshold=label_evidence_threshold,
        promotion_confidence=promotion_confidence,
        promotion_margin=promotion_margin,
        stereo_max_error_m=stereo_max_error_m,
        stereo_min_parallax_deg=stereo_min_parallax_deg,
    )


def make_track(
    module,
    track_id: int,
    x: float,
    y: float,
    *,
    z: float = 0.0,
    stamp_s: float = 10.0,
    hits: int = 4,
    state=None,
):
    if state is None:
        state = module.TrackState.CONFIRMED_UNLABELED
    return module.Track(
        track_id=track_id,
        position_xy=np.array([x, y], dtype=float),
        visualization_z=z,
        sonar_hit_count=hits,
        consecutive_misses=0,
        first_seen_stamp_s=stamp_s,
        last_sonar_stamp_s=stamp_s,
        last_sonar_stamp_msg=stamp_namespace(stamp_s),
        state=state,
    )


def make_observation(
    module,
    *,
    camera_name: str,
    origin_xy: tuple[float, float],
    target_xy: tuple[float, float],
    class_id: int = 1,
    confidence: float = 0.95,
    stamp_s: float = 10.0,
    half_width_deg: float = 1.0,
    origin_z: float = 0.0,
    direction_z: float = 0.0,
):
    origin = np.array([origin_xy[0], origin_xy[1], origin_z], dtype=float)
    planar_delta = np.asarray(target_xy, dtype=float) - np.asarray(origin_xy, dtype=float)
    planar_direction = planar_delta / np.linalg.norm(planar_delta)
    direction = np.array(
        [planar_direction[0], planar_direction[1], direction_z],
        dtype=float,
    )
    direction /= np.linalg.norm(direction)
    center_angle = float(np.arctan2(direction[1], direction[0]))
    half_width = np.deg2rad(half_width_deg)

    def direction_at(angle: float) -> np.ndarray:
        return np.array([np.cos(angle), np.sin(angle), direction[2]], dtype=float)

    stamp = make_stamp(stamp_s)
    return module.BearingObservation(
        camera_name=camera_name,
        stamp_s=stamp_s,
        stamp_key=(stamp.sec, stamp.nanosec),
        class_id=class_id,
        detector_confidence=confidence,
        origin_map=origin,
        center_direction_map=direction,
        left_direction_map=direction_at(center_angle + half_width),
        right_direction_map=direction_at(center_angle - half_width),
        center_angle_rad=center_angle,
        half_width_rad=half_width,
        bbox_width_px=20.0,
        bbox_height_px=40.0,
    )


def quaternion_from_rotation_matrix(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Return geometry quaternion components (x, y, z, w)."""
    matrix = np.asarray(rotation, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        diagonal_index = int(np.argmax(np.diag(matrix)))
        if diagonal_index == 0:
            scale = sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
        elif diagonal_index == 1:
            scale = sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
    return float(x), float(y), float(z), float(w)


def make_optical_to_map_transform(
    camera_frame: str,
    *,
    origin_xyz: tuple[float, float, float],
    map_frame: str = "map",
) -> TransformStamped:
    """Map optical (right, down, forward) into map (forward, left, up).

    The mapping is:
      map x = optical z
      map y = -optical x
      map z = -optical y
    """
    rotation = np.array(
        [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ],
        dtype=float,
    )
    qx, qy, qz, qw = quaternion_from_rotation_matrix(rotation)

    transform = TransformStamped()
    transform.header.frame_id = map_frame
    transform.child_frame_id = camera_frame
    transform.header.stamp = make_stamp(0.0)
    transform.transform.translation.x = float(origin_xyz[0])
    transform.transform.translation.y = float(origin_xyz[1])
    transform.transform.translation.z = float(origin_xyz[2])
    transform.transform.rotation.x = qx
    transform.transform.rotation.y = qy
    transform.transform.rotation.z = qz
    transform.transform.rotation.w = qw
    return transform


def make_camera_info(
    camera_frame: str,
    *,
    fx: float,
    fy: float | None = None,
    cx: float = 320.0,
    cy: float = 240.0,
    width: int = 640,
    height: int = 480,
) -> CameraInfo:
    if fy is None:
        fy = fx
    msg = CameraInfo()
    msg.header.frame_id = camera_frame
    msg.header.stamp = make_stamp(0.0)
    msg.width = width
    msg.height = height
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0] * 5
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


def make_detection(
    *,
    class_name: str,
    confidence: float,
    center_u: float,
    center_v: float = 240.0,
    width_px: float = 20.0,
    height_px: float = 40.0,
    frame_id: str,
    stamp_s: float,
) -> Detection2D:
    detection = Detection2D()
    detection.header.frame_id = frame_id
    detection.header.stamp = make_stamp(stamp_s)

    center = detection.bbox.center
    if hasattr(center, "position"):
        center.position.x = float(center_u)
        center.position.y = float(center_v)
    else:
        center.x = float(center_u)
        center.y = float(center_v)
    detection.bbox.size_x = float(width_px)
    detection.bbox.size_y = float(height_px)

    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = class_name
    result.hypothesis.score = float(confidence)
    detection.results = [result]
    return detection


def make_detection_array(
    detections: Iterable[Detection2D],
    *,
    frame_id: str,
    stamp_s: float,
) -> Detection2DArray:
    msg = Detection2DArray()
    msg.header.frame_id = frame_id
    msg.header.stamp = make_stamp(stamp_s)
    msg.detections = list(detections)
    for detection in msg.detections:
        detection.header = msg.header
    return msg


def make_cloud(points: Iterable[tuple[float, float, float]], *, stamp_s: float, frame_id: str = "map"):
    header = Header()
    header.frame_id = frame_id
    header.stamp = make_stamp(stamp_s)
    return pc2.create_cloud_xyz32(header, list(points))
