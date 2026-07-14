from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from enum import Enum, auto
from math import acos, atan2, cos, exp, log10, pi, sin
from typing import Iterable, Sequence

import geometry_msgs.msg
from geometry_msgs.msg import Point, PoseStamped
from image_geometry import PinholeCameraModel
import numpy as np
from pontus_bringup.topic_config import TopicConfig
from pontus_msgs.msg import SemanticObject
from pontus_msgs.srv import AddSemanticObject
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import ColorRGBA, Header
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2D, Detection2DArray


# -----------------------------------------------------------------------------
# Configuration and state objects
# -----------------------------------------------------------------------------


@dataclass(frozen=True)
class FusionSettings:
    map_frame: str = "map"

    # Sonar tracking.
    sonar_track_gate_m: float = 0.450
    confirmation_hits: int = 4
    tentative_max_misses: int = 3
    confirmed_max_misses: int = 12
    hard_max_age_without_sonar_s: float = 30.0
    position_mean_sample_cap: int = 20
    scatter_update_gain: float = 0.15

    # Camera-to-track association.
    camera_track_max_age_s: float = 5.0
    camera_age_decay_tau_s: float = 3.0
    camera_range_cost_weight: float = 0.10
    camera_persistence_bonus_weight: float = 0.05

    # Evidence and promotion.
    evidence_cap: float = 20.0
    strong_observation_confidence: float = 0.70
    strong_observation_min_bearing_quality: float = 0.50
    contradiction_hold_s: float = 2.0
    promoted_track_publish_period_s: float = 0.75

    # Stereo consistency.
    enable_stereo: bool = True
    stereo_sync_slop_s: float = 0.15
    stereo_condition_number_max: float = 1.0e4
    stereo_max_range_m: float = 10.0
    stereo_evidence_bonus: float = 0.50

    # Hybrid z and debug output.
    camera_z_max_age_s: float = 2.0
    debug_ray_length_m: float = 10.0


# -----------------------------------------------------------------------------
# General helpers
# -----------------------------------------------------------------------------


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def stamp_key(stamp) -> tuple[int, int]:
    return int(stamp.sec), int(stamp.nanosec)


def wrap_angle(angle_rad: float) -> float:
    return (angle_rad + pi) % (2.0 * pi) - pi


def cross_2d(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def quaternion_to_rotation_matrix(rotation) -> np.ndarray:
    """Return the 3x3 rotation matrix represented by a geometry quaternion."""
    x = float(rotation.x)
    y = float(rotation.y)
    z = float(rotation.z)
    w = float(rotation.w)

    norm = x * x + y * y + z * z + w * w
    if norm < 1e-15:
        return np.eye(3, dtype=float)

    scale = 2.0 / norm
    xx = x * x * scale
    yy = y * y * scale
    zz = z * z * scale
    xy = x * y * scale
    xz = x * z * scale
    yz = y * z * scale
    wx = w * x * scale
    wy = w * y * scale
    wz = w * z * scale

    return np.array(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ],
        dtype=float,
    )


class StampDeduplicator:
    """Bounded exact-stamp duplicate detection for one ROS topic."""

    def __init__(self, capacity: int = 256) -> None:
        self._capacity = capacity
        self._keys: set[tuple[int, int]] = set()
        self._order: deque[tuple[int, int]] = deque()

    def seen(self, stamp) -> bool:
        key = stamp_key(stamp)
        if key in self._keys:
            return True

        self._keys.add(key)
        self._order.append(key)
        if len(self._order) > self._capacity:
            old_key = self._order.popleft()
            self._keys.remove(old_key)
        return False


# -----------------------------------------------------------------------------
# Pure-Python rectangular Hungarian assignment
# -----------------------------------------------------------------------------


def _hungarian_rows_to_columns(cost: np.ndarray) -> list[int]:
    """Solve a rectangular minimum-cost assignment with rows <= columns.

    Returns one selected column for every row. This is the shortest augmenting
    path form of the Hungarian algorithm and has O(n^2 m) complexity.
    """
    if cost.ndim != 2:
        raise ValueError("cost matrix must be two-dimensional")

    n_rows, n_cols = cost.shape
    if n_rows == 0:
        return []
    if n_rows > n_cols:
        raise ValueError("Hungarian helper requires rows <= columns")
    if not np.all(np.isfinite(cost)):
        raise ValueError("Hungarian helper requires finite costs")

    u = np.zeros(n_rows + 1, dtype=float)
    v = np.zeros(n_cols + 1, dtype=float)
    p = np.zeros(n_cols + 1, dtype=int)
    way = np.zeros(n_cols + 1, dtype=int)

    for row in range(1, n_rows + 1):
        p[0] = row
        min_value = np.full(n_cols + 1, np.inf, dtype=float)
        used = np.zeros(n_cols + 1, dtype=bool)
        column_0 = 0

        while True:
            used[column_0] = True
            row_0 = p[column_0]
            delta = np.inf
            column_1 = 0

            for column in range(1, n_cols + 1):
                if used[column]:
                    continue
                reduced_cost = (
                    cost[row_0 - 1, column - 1]
                    - u[row_0]
                    - v[column]
                )
                if reduced_cost < min_value[column]:
                    min_value[column] = reduced_cost
                    way[column] = column_0
                if min_value[column] < delta:
                    delta = min_value[column]
                    column_1 = column

            for column in range(n_cols + 1):
                if used[column]:
                    u[p[column]] += delta
                    v[column] -= delta
                else:
                    min_value[column] -= delta

            column_0 = column_1
            if p[column_0] == 0:
                break

        while True:
            column_1 = way[column_0]
            p[column_0] = p[column_1]
            column_0 = column_1
            if column_0 == 0:
                break

    row_to_column = [-1] * n_rows
    for column in range(1, n_cols + 1):
        if p[column] != 0:
            row_to_column[p[column] - 1] = column - 1
    return row_to_column


def solve_gated_assignment(
    pair_costs: np.ndarray,
    valid_pairs: np.ndarray,
    unmatched_cost: float,
) -> list[tuple[int, int]]:
    """Solve a one-to-one assignment while permitting every row to be unmatched.

    Rows generally represent new measurements/detections and columns represent
    existing tracks. One private dummy column is added per row. Invalid pairs
    are more expensive than choosing a dummy, so they can never be accepted.
    """
    if pair_costs.shape != valid_pairs.shape:
        raise ValueError(
            "pair_costs and valid_pairs must have matching shapes")

    n_rows, n_real_columns = pair_costs.shape
    if n_rows == 0 or n_real_columns == 0:
        return []

    forbidden_cost = unmatched_cost + 1.0e6
    augmented = np.full(
        (n_rows, n_real_columns + n_rows),
        forbidden_cost,
        dtype=float,
    )
    augmented[:, :n_real_columns] = np.where(
        valid_pairs,
        pair_costs,
        forbidden_cost,
    )

    # Each row owns one dummy column. A tiny deterministic offset prevents
    # arbitrary dummy permutations without materially changing the solution.
    for row in range(n_rows):
        augmented[row, n_real_columns + row] = unmatched_cost + row * 1e-12

    row_to_column = _hungarian_rows_to_columns(augmented)
    accepted: list[tuple[int, int]] = []
    for row, column in enumerate(row_to_column):
        if column < n_real_columns and valid_pairs[row, column]:
            accepted.append((row, column))
    return accepted




@dataclass(frozen=True)
class ClassFusionConfig:
    semantic_id: int
    min_detector_confidence: float
    min_bbox_width_px: float
    min_bbox_height_px: float
    bearing_margin_deg: float
    max_track_range_m: float
    label_evidence_threshold: float
    promotion_confidence: float
    promotion_margin: float
    stereo_max_error_m: float | None
    stereo_min_parallax_deg: float

    @property
    def bearing_margin_rad(self) -> float:
        return self.bearing_margin_deg * pi / 180.0


@dataclass
class BearingObservation:
    camera_name: str
    stamp_s: float
    stamp_key: tuple[int, int]
    class_id: int
    detector_confidence: float
    origin_map: np.ndarray          # shape (3,)
    center_direction_map: np.ndarray  # shape (3,), unit length
    left_direction_map: np.ndarray
    right_direction_map: np.ndarray
    center_angle_rad: float
    half_width_rad: float
    bbox_width_px: float
    bbox_height_px: float
    bearing_residual_rad: float | None = None
    normalized_bearing_residual: float | None = None
    association_cost: float | None = None


class TrackState(Enum):
    TENTATIVE = auto()
    CONFIRMED_UNLABELED = auto()
    LABELED = auto()
    RETIRED = auto()


@dataclass
class Track:
    track_id: int
    position_xy: np.ndarray
    visualization_z: float
    sonar_hit_count: int
    consecutive_misses: int
    first_seen_stamp_s: float
    last_sonar_stamp_s: float
    last_sonar_stamp_msg: object
    state: TrackState = TrackState.TENTATIVE
    effective_position_samples: int = 1
    scatter_xy: np.ndarray = field(
        default_factory=lambda: np.zeros((2, 2), dtype=float)
    )
    label_evidence: dict[int, float] = field(default_factory=dict)
    last_camera_observations: dict[str, BearingObservation] = field(
        default_factory=dict
    )
    camera_z_estimates: dict[str, tuple[float, float]] = field(
        default_factory=dict
    )
    last_stereo_pair_key: tuple | None = None
    last_strong_label_id: int | None = None
    last_strong_label_stamp_s: float = -np.inf
    promoted_label: int | None = None
    last_semantic_publish_stamp_s: float = -np.inf

    def update_from_sonar(
        self,
        point_xyz: np.ndarray,
        stamp_s: float,
        stamp_msg,
        settings: FusionSettings,
    ) -> None:
        measurement_xy = np.asarray(point_xyz[:2], dtype=float)
        innovation = measurement_xy - self.position_xy

        self.effective_position_samples = min(
            self.effective_position_samples + 1,
            settings.position_mean_sample_cap,
        )
        alpha = 1.0 / float(self.effective_position_samples)
        self.position_xy = self.position_xy + alpha * innovation

        beta = settings.scatter_update_gain
        self.scatter_xy = (
            (1.0 - beta) * self.scatter_xy
            + beta * np.outer(innovation, innovation)
        )

        self.visualization_z = float(point_xyz[2])
        self.sonar_hit_count += 1
        self.consecutive_misses = 0
        self.last_sonar_stamp_s = stamp_s
        self.last_sonar_stamp_msg = stamp_msg

    def add_label_evidence(
        self,
        class_id: int,
        amount: float,
        settings: FusionSettings,
    ) -> None:
        current = self.label_evidence.get(class_id, 0.0)
        self.label_evidence[class_id] = min(
            settings.evidence_cap,
            current + max(0.0, amount),
        )

    def best_label_statistics(self) -> tuple[int | None, float, float, float]:
        """Return (label, evidence, normalized confidence, best-second margin)."""
        if not self.label_evidence:
            return None, 0.0, 0.0, 0.0

        ordered = sorted(
            self.label_evidence.items(),
            key=lambda item: item[1],
            reverse=True,
        )
        best_label, best_evidence = ordered[0]
        second_evidence = ordered[1][1] if len(ordered) > 1 else 0.0
        total = sum(self.label_evidence.values())
        confidence = best_evidence / max(total, 1e-12)
        return (
            best_label,
            float(best_evidence),
            float(confidence),
            float(best_evidence - second_evidence),
        )

    def display_z(self, reference_stamp_s: float, settings: FusionSettings) -> float:
        recent_values = [
            z_value
            for stamp_s, z_value in self.camera_z_estimates.values()
            if abs(reference_stamp_s - stamp_s) <= settings.camera_z_max_age_s
            and np.isfinite(z_value)
        ]
        if recent_values:
            return float(np.median(np.asarray(recent_values, dtype=float)))
        return float(self.visualization_z)


# -----------------------------------------------------------------------------
# Camera calibration and bearing conversion
# -----------------------------------------------------------------------------


@dataclass
class CameraContext:
    name: str
    configured_frame_id: str
    model: PinholeCameraModel = field(default_factory=PinholeCameraModel)
    calibration_ready: bool = False
    optical_frame_id: str = ""
    duplicate_filter: StampDeduplicator = field(
        default_factory=lambda: StampDeduplicator(256)
    )
    last_processed_stamp_s: float = -np.inf
    debug_publisher: object | None = None

    def update_camera_info(self, msg: CameraInfo) -> bool:
        was_ready = self.calibration_ready
        self.model.fromCameraInfo(msg)
        self.optical_frame_id = msg.header.frame_id or self.configured_frame_id
        self.calibration_ready = True
        return not was_ready

    @staticmethod
    def _bbox_center_and_size(detection: Detection2D) -> tuple[float, float, float, float]:
        center = detection.bbox.center
        if hasattr(center, "position"):
            center_x = float(center.position.x)
            center_y = float(center.position.y)
        else:
            center_x = float(center.x)
            center_y = float(center.y)
        return (
            center_x,
            center_y,
            float(detection.bbox.size_x),
            float(detection.bbox.size_y),
        )

    def detection_to_bearing(
        self,
        detection: Detection2D,
        class_id: int,
        confidence: float,
        stamp,
        transform,
    ) -> BearingObservation | None:
        center_u, center_v, width_px, height_px = self._bbox_center_and_size(
            detection
        )
        left_u = center_u - 0.5 * width_px
        right_u = center_u + 0.5 * width_px

        pixels = (
            np.array([center_u, center_v], dtype=np.float64),
            np.array([left_u, center_v], dtype=np.float64),
            np.array([right_u, center_v], dtype=np.float64),
        )

        rotation_matrix = quaternion_to_rotation_matrix(
            transform.transform.rotation)
        origin_map = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=float,
        )

        map_rays: list[np.ndarray] = []
        for pixel in pixels:
            rectified = self.model.rectifyPoint(pixel)
            ray_camera = np.asarray(
                self.model.projectPixelTo3dRay(rectified),
                dtype=float,
            )
            ray_map = rotation_matrix @ ray_camera
            norm_3d = float(np.linalg.norm(ray_map))
            if norm_3d < 1e-10:
                return None
            ray_map = ray_map / norm_3d

            norm_xy = float(np.linalg.norm(ray_map[:2]))
            if norm_xy < 1e-10:
                return None
            map_rays.append(ray_map)

        center_ray, left_ray, right_ray = map_rays
        center_angle = atan2(center_ray[1], center_ray[0])
        left_angle = atan2(left_ray[1], left_ray[0])
        right_angle = atan2(right_ray[1], right_ray[0])
        half_width = max(
            abs(wrap_angle(left_angle - center_angle)),
            abs(wrap_angle(right_angle - center_angle)),
        )

        return BearingObservation(
            camera_name=self.name,
            stamp_s=stamp_to_seconds(stamp),
            stamp_key=stamp_key(stamp),
            class_id=class_id,
            detector_confidence=confidence,
            origin_map=origin_map,
            center_direction_map=center_ray,
            left_direction_map=left_ray,
            right_direction_map=right_ray,
            center_angle_rad=center_angle,
            half_width_rad=half_width,
            bbox_width_px=width_px,
            bbox_height_px=height_px,
        )


# -----------------------------------------------------------------------------
# Persistent SONOPTIX track manager
# -----------------------------------------------------------------------------


class TrackManager:
    def __init__(self, settings: FusionSettings) -> None:
        self.settings = settings
        self.tracks: dict[int, Track] = {}
        self.next_track_id = 0

    def active_tracks(self) -> list[Track]:
        return [
            track
            for track in self.tracks.values()
            if track.state is not TrackState.RETIRED
        ]

    def confirmed_unlabeled_tracks(self) -> list[Track]:
        return [
            track
            for track in self.tracks.values()
            if track.state is TrackState.CONFIRMED_UNLABELED
        ]

    def process_sonar_centroids(
        self,
        points_xyz: np.ndarray,
        stamp_s: float,
        stamp_msg,
    ) -> tuple[list[Track], list[Track]]:
        """Update tracks and return (newly_confirmed, retired)."""
        active_tracks = self.active_tracks()
        n_points = len(points_xyz)
        n_tracks = len(active_tracks)

        if n_points > 0 and n_tracks > 0:
            costs = np.zeros((n_points, n_tracks), dtype=float)
            valid = np.zeros((n_points, n_tracks), dtype=bool)
            for point_index, point in enumerate(points_xyz):
                point_xy = np.asarray(point[:2], dtype=float)
                for track_index, track in enumerate(active_tracks):
                    distance = float(np.linalg.norm(
                        point_xy - track.position_xy))
                    costs[point_index, track_index] = distance
                    valid[point_index, track_index] = (
                        distance <= self.settings.sonar_track_gate_m
                    )

            assignments = solve_gated_assignment(
                pair_costs=costs,
                valid_pairs=valid,
                unmatched_cost=self.settings.sonar_track_gate_m + 1e-6,
            )
        else:
            assignments = []

        matched_point_indices = {pair[0] for pair in assignments}
        matched_track_indices = {pair[1] for pair in assignments}

        for point_index, track_index in assignments:
            active_tracks[track_index].update_from_sonar(
                points_xyz[point_index],
                stamp_s,
                stamp_msg,
                self.settings,
            )

        for point_index, point in enumerate(points_xyz):
            if point_index in matched_point_indices:
                continue
            self._create_track(point, stamp_s, stamp_msg)

        for track_index, track in enumerate(active_tracks):
            if track_index not in matched_track_indices:
                track.consecutive_misses += 1

        newly_confirmed: list[Track] = []
        retired: list[Track] = []
        for track in list(self.active_tracks()):
            if (
                track.state is TrackState.TENTATIVE
                and track.sonar_hit_count >= self.settings.confirmation_hits
            ):
                track.state = TrackState.CONFIRMED_UNLABELED
                newly_confirmed.append(track)

            max_misses = (
                self.settings.tentative_max_misses
                if track.state is TrackState.TENTATIVE
                else self.settings.confirmed_max_misses
            )
            too_many_misses = track.consecutive_misses > max_misses
            too_old = (
                stamp_s - track.last_sonar_stamp_s
                > self.settings.hard_max_age_without_sonar_s
            )

            if too_many_misses or too_old:
                track.state = TrackState.RETIRED
                retired.append(track)

        for track in retired:
            self.tracks.pop(track.track_id, None)

        return newly_confirmed, retired

    def _create_track(self, point_xyz: np.ndarray, stamp_s: float, stamp_msg) -> Track:
        track = Track(
            track_id=self.next_track_id,
            position_xy=np.asarray(point_xyz[:2], dtype=float).copy(),
            visualization_z=float(point_xyz[2]),
            sonar_hit_count=1,
            consecutive_misses=0,
            first_seen_stamp_s=stamp_s,
            last_sonar_stamp_s=stamp_s,
            last_sonar_stamp_msg=stamp_msg,
        )
        self.tracks[track.track_id] = track
        self.next_track_id += 1
        return track


# -----------------------------------------------------------------------------
# Per-camera gated assignment
# -----------------------------------------------------------------------------


class CameraAssociator:
    def __init__(self, settings: FusionSettings) -> None:
        self.settings = settings

    def assign(
        self,
        observations: Sequence[BearingObservation],
        tracks: Sequence[Track],
        class_configs: dict[int, ClassFusionConfig],
        default_config: ClassFusionConfig,
    ) -> list[tuple[BearingObservation, Track]]:
        if not observations or not tracks:
            return []

        costs = np.zeros((len(observations), len(tracks)), dtype=float)
        valid = np.zeros_like(costs, dtype=bool)

        for observation_index, observation in enumerate(observations):
            config = class_configs.get(observation.class_id, default_config)
            origin_xy = observation.origin_map[:2]

            for track_index, track in enumerate(tracks):
                age_s = abs(observation.stamp_s - track.last_sonar_stamp_s)
                if age_s > self.settings.camera_track_max_age_s:
                    continue

                delta_xy = track.position_xy - origin_xy
                range_m = float(np.linalg.norm(delta_xy))
                if range_m < 1e-8 or range_m > config.max_track_range_m:
                    continue

                predicted_direction = delta_xy / range_m
                center_direction_xy = observation.center_direction_map[:2]
                center_direction_xy = center_direction_xy / np.linalg.norm(
                    center_direction_xy
                )

                # Must be in the forward half-plane of the actual detection ray.
                if float(delta_xy.dot(center_direction_xy)) <= 0.0:
                    continue

                predicted_angle = atan2(
                    predicted_direction[1], predicted_direction[0])
                residual = wrap_angle(
                    predicted_angle - observation.center_angle_rad)
                allowed_half_width = (
                    observation.half_width_rad + config.bearing_margin_rad
                )
                if abs(residual) > allowed_half_width:
                    continue

                normalized_residual = abs(
                    residual) / max(allowed_half_width, 1e-6)
                persistence = min(
                    1.0,
                    track.sonar_hit_count /
                    float(self.settings.confirmation_hits),
                )
                cost = (
                    normalized_residual
                    + self.settings.camera_range_cost_weight
                    * (range_m / max(config.max_track_range_m, 1e-6))
                    - self.settings.camera_persistence_bonus_weight * persistence
                )

                costs[observation_index, track_index] = max(0.0, cost)
                valid[observation_index, track_index] = True

        assignments = solve_gated_assignment(
            pair_costs=costs,
            valid_pairs=valid,
            unmatched_cost=1.25,
        )

        accepted: list[tuple[BearingObservation, Track]] = []
        for observation_index, track_index in assignments:
            observation = observations[observation_index]
            track = tracks[track_index]

            delta_xy = track.position_xy - observation.origin_map[:2]
            predicted_angle = atan2(delta_xy[1], delta_xy[0])
            residual = wrap_angle(
                predicted_angle - observation.center_angle_rad)
            config = class_configs.get(observation.class_id, default_config)
            allowed_half_width = (
                observation.half_width_rad + config.bearing_margin_rad
            )

            observation.bearing_residual_rad = residual
            observation.normalized_bearing_residual = (
                abs(residual) / max(allowed_half_width, 1e-6)
            )
            observation.association_cost = float(
                costs[observation_index, track_index])
            accepted.append((observation, track))

        return accepted


# -----------------------------------------------------------------------------
# Two-bearing planar stereo consistency
# -----------------------------------------------------------------------------


@dataclass(frozen=True)
class StereoResult:
    point_xy: np.ndarray
    parallax_deg: float
    condition_number: float
    sonar_error_m: float
    quality: float


class StereoTriangulator:
    def __init__(self, settings: FusionSettings) -> None:
        self.settings = settings

    def triangulate(
        self,
        first: BearingObservation,
        second: BearingObservation,
        track: Track,
        class_config: ClassFusionConfig,
    ) -> StereoResult | None:
        if class_config.stereo_max_error_m is None:
            return None

        c1 = np.asarray(first.origin_map[:2], dtype=float)
        c2 = np.asarray(second.origin_map[:2], dtype=float)
        d1 = np.asarray(first.center_direction_map[:2], dtype=float)
        d2 = np.asarray(second.center_direction_map[:2], dtype=float)
        d1 /= np.linalg.norm(d1)
        d2 /= np.linalg.norm(d2)

        dot_abs = clamp(abs(float(d1.dot(d2))), 0.0, 1.0)
        parallax_rad = acos(dot_abs)
        parallax_deg = parallax_rad * 180.0 / pi
        if parallax_deg < class_config.stereo_min_parallax_deg:
            return None

        n1 = np.array([-d1[1], d1[0]], dtype=float)
        n2 = np.array([-d2[1], d2[0]], dtype=float)

        # Narrower sectors receive slightly more weight.
        width1 = max(first.half_width_rad, 0.5 * pi / 180.0)
        width2 = max(second.half_width_rad, 0.5 * pi / 180.0)
        w1 = 1.0 / (width1 * width1)
        w2 = 1.0 / (width2 * width2)

        hessian = w1 * np.outer(n1, n1) + w2 * np.outer(n2, n2)
        condition_number = float(np.linalg.cond(hessian))
        if (
            not np.isfinite(condition_number)
            or condition_number > self.settings.stereo_condition_number_max
        ):
            return None

        gradient = (
            w1 * np.outer(n1, n1) @ c1
            + w2 * np.outer(n2, n2) @ c2
        )
        try:
            point_xy = np.linalg.solve(hessian, gradient)
        except np.linalg.LinAlgError:
            return None

        forward_1 = float((point_xy - c1).dot(d1))
        forward_2 = float((point_xy - c2).dot(d2))
        if forward_1 <= 0.0 or forward_2 <= 0.0:
            return None
        if (
            forward_1 > self.settings.stereo_max_range_m
            or forward_2 > self.settings.stereo_max_range_m
        ):
            return None

        sonar_error_m = float(np.linalg.norm(point_xy - track.position_xy))
        if sonar_error_m > class_config.stereo_max_error_m:
            return None

        q_parallax = clamp(parallax_deg / 15.0, 0.0, 1.0)
        max_condition_log = max(
            log10(self.settings.stereo_condition_number_max),
            1e-6,
        )
        q_condition = clamp(
            1.0 - log10(max(condition_number, 1.0)) / max_condition_log,
            0.0,
            1.0,
        )
        sigma_error = max(0.5 * class_config.stereo_max_error_m, 1e-3)
        q_error = exp(-0.5 * (sonar_error_m / sigma_error) ** 2)
        quality = clamp(q_parallax * q_condition * q_error, 0.0, 1.0)

        return StereoResult(
            point_xy=point_xy,
            parallax_deg=parallax_deg,
            condition_number=condition_number,
            sonar_error_m=sonar_error_m,
            quality=quality,
        )


# -----------------------------------------------------------------------------
# Fusion coordinator
# -----------------------------------------------------------------------------


class FusionCoordinator:
    def __init__(
        self,
        settings: FusionSettings,
        track_manager: TrackManager,
        camera_associator: CameraAssociator,
        stereo_triangulator: StereoTriangulator,
        class_configs: dict[int, ClassFusionConfig],
        default_class_config: ClassFusionConfig,
        camera_names: Sequence[str],
    ) -> None:
        self.settings = settings
        self.track_manager = track_manager
        self.camera_associator = camera_associator
        self.stereo_triangulator = stereo_triangulator
        self.class_configs = class_configs
        self.default_class_config = default_class_config
        self.camera_names = tuple(camera_names)

    def process_camera_observations(
        self,
        observations: Sequence[BearingObservation],
    ) -> tuple[list[tuple[BearingObservation, Track]], list[Track]]:
        tracks = self.track_manager.active_tracks()
        assignments = self.camera_associator.assign(
            observations,
            tracks,
            self.class_configs,
            self.default_class_config,
        )

        promoted_tracks: list[Track] = []
        for observation, track in assignments:
            self._apply_camera_evidence(track, observation)
            self._update_hybrid_z(track, observation)
            track.last_camera_observations[observation.camera_name] = observation
            self._try_stereo_bonus(track, observation)

            if self._try_promote(track, observation.stamp_s):
                promoted_tracks.append(track)

        return assignments, promoted_tracks

    def _apply_camera_evidence(
        self,
        track: Track,
        observation: BearingObservation,
    ) -> None:
        normalized_residual = observation.normalized_bearing_residual
        if normalized_residual is None:
            return

        q_detector = clamp(observation.detector_confidence, 0.0, 1.0)
        q_bearing = clamp(1.0 - normalized_residual, 0.0, 1.0)
        age_s = abs(observation.stamp_s - track.last_sonar_stamp_s)
        q_age = exp(-age_s / max(self.settings.camera_age_decay_tau_s, 1e-6))
        evidence = q_detector * q_bearing * q_age
        track.add_label_evidence(observation.class_id, evidence, self.settings)

        if (
            q_detector >= self.settings.strong_observation_confidence
            and q_bearing >= self.settings.strong_observation_min_bearing_quality
        ):
            track.last_strong_label_id = observation.class_id
            track.last_strong_label_stamp_s = observation.stamp_s

    @staticmethod
    def _ray_z_at_track_xy(
        observation: BearingObservation,
        track_xy: np.ndarray,
    ) -> float | None:
        direction_xy = observation.center_direction_map[:2]
        denominator = float(direction_xy.dot(direction_xy))
        if denominator < 1e-10:
            return None

        t = float(
            direction_xy.dot(track_xy - observation.origin_map[:2])
            / denominator
        )
        if t < 0.0:
            return None
        return float(
            observation.origin_map[2]
            + t * observation.center_direction_map[2]
        )

    def _update_hybrid_z(
        self,
        track: Track,
        observation: BearingObservation,
    ) -> None:
        z_estimate = self._ray_z_at_track_xy(observation, track.position_xy)
        if z_estimate is not None and np.isfinite(z_estimate):
            track.camera_z_estimates[observation.camera_name] = (
                observation.stamp_s,
                z_estimate,
            )

    def _try_stereo_bonus(
        self,
        track: Track,
        newest_observation: BearingObservation,
    ) -> None:
        if not self.settings.enable_stereo or len(self.camera_names) < 2:
            return

        other_observations = [
            observation
            for camera_name, observation in track.last_camera_observations.items()
            if camera_name != newest_observation.camera_name
        ]
        if not other_observations:
            return

        # Select the temporally nearest observation from another camera.
        other = min(
            other_observations,
            key=lambda candidate: abs(
                candidate.stamp_s - newest_observation.stamp_s
            ),
        )
        if other.class_id != newest_observation.class_id:
            return
        if (
            abs(other.stamp_s - newest_observation.stamp_s)
            > self.settings.stereo_sync_slop_s
        ):
            return

        pair_key = tuple(
            sorted(
                (
                    (other.camera_name, other.stamp_key),
                    (newest_observation.camera_name, newest_observation.stamp_key),
                )
            )
        )
        if pair_key == track.last_stereo_pair_key:
            return
        track.last_stereo_pair_key = pair_key

        config = self.class_configs.get(
            newest_observation.class_id,
            self.default_class_config,
        )
        result = self.stereo_triangulator.triangulate(
            other,
            newest_observation,
            track,
            config,
        )
        if result is None:
            return

        bonus = self.settings.stereo_evidence_bonus * result.quality
        track.add_label_evidence(
            newest_observation.class_id,
            bonus,
            self.settings,
        )

    def _try_promote(self, track: Track, observation_stamp_s: float) -> bool:
        if track.state is not TrackState.CONFIRMED_UNLABELED:
            return False

        best_label, evidence, confidence, margin = track.best_label_statistics()
        if best_label is None:
            return False
        config = self.class_configs.get(best_label, self.default_class_config)

        recent_contradiction = (
            track.last_strong_label_id is not None
            and track.last_strong_label_id != best_label
            and observation_stamp_s - track.last_strong_label_stamp_s
            <= self.settings.contradiction_hold_s
        )

        if (
            evidence >= config.label_evidence_threshold
            and confidence >= config.promotion_confidence
            and margin >= config.promotion_margin
            and not recent_contradiction
        ):
            track.state = TrackState.LABELED
            track.promoted_label = best_label
            return True
        return False


# -----------------------------------------------------------------------------
# ROS 2 node
# -----------------------------------------------------------------------------


class ImageCoordinator(Node):
    def __init__(self) -> None:
        super().__init__("image_to_cluster_coordinator")


        self.topics = TopicConfig(
            self,
            [
                "cameras",
                "camera_frame_template",
                "camera_info_topic_template",
                "detections_topic_template",
                "semantic_object_service",
                "pointcloud_cluster_topic",
            ],
        )

        self.settings = FusionSettings()
        self.name_map = self._build_name_map()
        self.class_configs, self.default_class_config = self._build_class_configs()
        self._validate_configuration()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.track_manager = TrackManager(self.settings)
        self.camera_associator = CameraAssociator(self.settings)
        self.stereo_triangulator = StereoTriangulator(self.settings)
        self.fusion = FusionCoordinator(
            settings=self.settings,
            track_manager=self.track_manager,
            camera_associator=self.camera_associator,
            stereo_triangulator=self.stereo_triangulator,
            class_configs=self.class_configs,
            default_class_config=self.default_class_config,
            camera_names=self.topics.cameras,
        )

        self.semantic_client = self.create_client(
            AddSemanticObject,
            self.topics.semantic_object_service,
        )
        while not self.semantic_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        qos = QoSProfile(depth=10)
        self.sonar_duplicate_filter = StampDeduplicator(256)
        self.last_sonar_stamp_s = -np.inf
        self.latest_sonar_stamp_msg = None

        self.unlabeled_tracks_publisher = self.create_publisher(
            PointCloud2,
            "/pontus/unlabeled_candidate_tracks",
            10,
        )

        self.camera_contexts: dict[str, CameraContext] = {}
        self.info_subscriptions: list[object] = []
        self.detection_subscriptions: list[object] = []

        for camera_name in self.topics.cameras:
            configured_frame = self.topics.camera_frame_template.replace(
                "{camera}", camera_name
            )
            context = CameraContext(
                name=camera_name,
                configured_frame_id=configured_frame,
            )
            context.debug_publisher = self.create_publisher(
                MarkerArray,
                f"/pontus/{camera_name}/debug_lines",
                10,
            )
            self.camera_contexts[camera_name] = context

            info_topic = self.topics.camera_info_topic_template.replace(
                "{camera}", camera_name
            )
            detection_topic = self.topics.detections_topic_template.replace(
                "{camera}", camera_name
            )

            self.info_subscriptions.append(
                self.create_subscription(
                    CameraInfo,
                    info_topic,
                    lambda msg, name=camera_name: self.camera_info_callback(
                        name, msg
                    ),
                    qos,
                )
            )
            self.detection_subscriptions.append(
                self.create_subscription(
                    Detection2DArray,
                    detection_topic,
                    lambda msg, name=camera_name: self.camera_callback(
                        name, msg),
                    qos,
                )
            )
            self.get_logger().info(
                f"Camera '{camera_name}' detections: {detection_topic}"
            )
            self.get_logger().info(
                f"Camera '{camera_name}' calibration: {info_topic}"
            )

        self.sonar_subscription = self.create_subscription(
            PointCloud2,
            self.topics.pointcloud_cluster_topic,
            self.sonar_callback,
            qos,
        )
        self.get_logger().info(
            f"Cluster topic subscribed: {self.topics.pointcloud_cluster_topic}"
        )
        self.get_logger().info(
            "Unlabeled confirmed tracks publish as PointCloud2 on "
            "'/pontus/unlabeled_candidate_tracks'."
        )

        self.get_logger().info("Starting up Sonar / Image coordination 12")

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    @staticmethod
    def _build_name_map() -> dict[str, int]:
        return {
            "gate_side": SemanticObject.GATE_LEFT,
            "red_slalom": SemanticObject.SLALOM_RED,
            "reef_shark": SemanticObject.GATE_IMAGE_SHARK,
            "saw_shark": SemanticObject.GATE_IMAGE_FISH,
            "white_slalom": SemanticObject.SLALOM_WHITE,
            "path_marker": SemanticObject.PATH_MARKER,
            "vertical pole": SemanticObject.VERTICAL_MARKER,
            # Simulation aliases.
            "slalom_red": SemanticObject.SLALOM_RED,
            "slalom_white": SemanticObject.SLALOM_WHITE,
            "gate_shark": SemanticObject.GATE_IMAGE_SHARK,
            "gate_fish": SemanticObject.GATE_IMAGE_FISH,
            "left_gate": SemanticObject.GATE_LEFT,
            "right_gate": SemanticObject.GATE_LEFT,
            "vertical_marker": SemanticObject.VERTICAL_MARKER,
            # Simulator numeric labels.
            str(SemanticObject.SLALOM_RED): SemanticObject.SLALOM_RED,
            str(SemanticObject.SLALOM_WHITE): SemanticObject.SLALOM_WHITE,
            str(SemanticObject.GATE_IMAGE_SHARK): SemanticObject.GATE_IMAGE_SHARK,
            str(SemanticObject.GATE_IMAGE_FISH): SemanticObject.GATE_IMAGE_FISH,
            str(SemanticObject.GATE_LEFT): SemanticObject.GATE_LEFT,
            str(SemanticObject.GATE_RIGHT): SemanticObject.GATE_LEFT,
            str(SemanticObject.VERTICAL_MARKER): SemanticObject.VERTICAL_MARKER,
            str(SemanticObject.BIN): SemanticObject.BIN,
            str(SemanticObject.OCTAGON): SemanticObject.OCTAGON,
            str(SemanticObject.TARGET): SemanticObject.TARGET,
            str(SemanticObject.PATH_MARKER): SemanticObject.PATH_MARKER,
        }

    @staticmethod
    def _build_class_configs(
    ) -> tuple[dict[int, ClassFusionConfig], ClassFusionConfig]:
        """Centralized class-specific fusion parameters.

        These are conservative competition starting points. They intentionally
        omit all physical-size and monocular-range parameters from Version 1.
        """

        def config(
            semantic_id: int,
            min_bbox_width_px: float,
            min_bbox_height_px: float,
            bearing_margin_deg: float,
            max_track_range_m: float,
            stereo_max_error_m: float,
        ) -> ClassFusionConfig:
            return ClassFusionConfig(
                semantic_id=semantic_id,
                min_detector_confidence=0.40,
                min_bbox_width_px=min_bbox_width_px,
                min_bbox_height_px=min_bbox_height_px,
                bearing_margin_deg=bearing_margin_deg,
                max_track_range_m=max_track_range_m,
                label_evidence_threshold=2.50,
                promotion_confidence=0.65,
                promotion_margin=1.00,
                stereo_max_error_m=stereo_max_error_m,
                stereo_min_parallax_deg=3.0,
            )

        configs = {
            SemanticObject.GATE_LEFT: config(
                SemanticObject.GATE_LEFT, 6.0, 8.0, 5.0, 5.0, 1.25
            ),
            SemanticObject.SLALOM_RED: config(
                SemanticObject.SLALOM_RED, 3.0, 8.0, 6.0, 5.0, 1.25
            ),
            SemanticObject.SLALOM_WHITE: config(
                SemanticObject.SLALOM_WHITE, 3.0, 8.0, 6.0, 5.0, 1.25
            ),
            SemanticObject.GATE_IMAGE_SHARK: config(
                SemanticObject.GATE_IMAGE_SHARK, 8.0, 8.0, 3.0, 5.0, 0.90
            ),
            SemanticObject.GATE_IMAGE_FISH: config(
                SemanticObject.GATE_IMAGE_FISH, 8.0, 8.0, 3.0, 5.0, 0.90
            ),
            SemanticObject.PATH_MARKER: config(
                SemanticObject.PATH_MARKER, 8.0, 5.0, 4.0, 5.0, 1.00
            ),
            SemanticObject.VERTICAL_MARKER: config(
                SemanticObject.VERTICAL_MARKER, 3.0, 8.0, 6.0, 5.0, 1.00
            ),
            SemanticObject.BIN: config(
                SemanticObject.BIN, 8.0, 8.0, 4.0, 5.0, 1.00
            ),
            SemanticObject.OCTAGON: config(
                SemanticObject.OCTAGON, 12.0, 12.0, 7.0, 7.0, 1.75
            ),
            SemanticObject.TARGET: config(
                SemanticObject.TARGET, 8.0, 8.0, 4.0, 5.0, 1.00
            ),
        }

        default = ClassFusionConfig(
            semantic_id=-1,
            min_detector_confidence=0.40,
            min_bbox_width_px=5.0,
            min_bbox_height_px=5.0,
            bearing_margin_deg=6.0,
            max_track_range_m=5.0,
            label_evidence_threshold=2.50,
            promotion_confidence=0.65,
            promotion_margin=1.00,
            stereo_max_error_m=1.25,
            stereo_min_parallax_deg=3.0,
        )
        return configs, default

    def _validate_configuration(self) -> None:
        if len(set(self.topics.cameras)) != len(self.topics.cameras):
            raise ValueError("Configured camera names must be distinct")
        if len(self.topics.cameras) < 1:
            raise ValueError("At least one camera must be configured")

        all_configs = list(self.class_configs.values()) + [
            self.default_class_config
        ]
        for config in all_configs:
            if not 0.0 <= config.min_detector_confidence <= 1.0:
                raise ValueError(
                    "Detector confidence thresholds must be in [0, 1]")
            if not 0.0 <= config.promotion_confidence <= 1.0:
                raise ValueError(
                    "Promotion confidence thresholds must be in [0, 1]")
            if config.min_bbox_width_px < 0.0 or config.min_bbox_height_px < 0.0:
                raise ValueError("Bounding-box thresholds must be nonnegative")
            if config.bearing_margin_deg < 0.0:
                raise ValueError("Bearing margins must be nonnegative")
            if config.max_track_range_m <= 0.0:
                raise ValueError("Maximum track ranges must be positive")
            if config.promotion_margin < 0.0:
                raise ValueError("Promotion margins must be nonnegative")
            if not 0.0 <= config.stereo_min_parallax_deg <= 90.0:
                raise ValueError(
                    "Stereo parallax threshold must be in [0, 90] deg")

        known_ids = set(self.class_configs)
        unknown_mappings = {
            class_id for class_id in self.name_map.values() if class_id not in known_ids
        }
        if unknown_mappings:
            raise ValueError(
                f"Detector mappings reference classes without config: {unknown_mappings}"
            )

    # ------------------------------------------------------------------
    # Sonar path: exactly one update per accepted cloud
    # ------------------------------------------------------------------

    def sonar_callback(self, msg: PointCloud2) -> None:
        stamp_s = stamp_to_seconds(msg.header.stamp)
        if self.sonar_duplicate_filter.seen(msg.header.stamp):
            self.get_logger().warn(
                f"Ignoring duplicate sonar stamp {stamp_key(msg.header.stamp)}"
            )
            return
        if stamp_s <= self.last_sonar_stamp_s:
            self.get_logger().warn(
                "Ignoring out-of-order sonar cloud: "
                f"{stamp_s:.9f} <= {self.last_sonar_stamp_s:.9f}"
            )
            return

        transformed = self._transform_cloud_to_map(msg)
        if transformed is None:
            return

        points_xyz = pc2.read_points_numpy(
            transformed,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )
        points_xyz = np.asarray(points_xyz, dtype=float)
        if points_xyz.size == 0:
            points_xyz = np.empty((0, 3), dtype=float)
        else:
            points_xyz = points_xyz.reshape((-1, 3))
            points_xyz = points_xyz[np.all(np.isfinite(points_xyz), axis=1)]

        self.track_manager.process_sonar_centroids(
            points_xyz=points_xyz,
            stamp_s=stamp_s,
            stamp_msg=transformed.header.stamp,
        )
        self.last_sonar_stamp_s = stamp_s
        self.latest_sonar_stamp_msg = transformed.header.stamp
        self._publish_unlabeled_track_snapshot(transformed.header.stamp)

    # ------------------------------------------------------------------
    # Camera paths: independent per-camera processing
    # ------------------------------------------------------------------

    def camera_info_callback(self, camera_name: str, msg: CameraInfo) -> None:
        context = self.camera_contexts[camera_name]
        first_initialization = context.update_camera_info(msg)
        if first_initialization:
            self.get_logger().info(
                f"Camera model initialized for '{camera_name}' in frame "
                f"'{context.optical_frame_id}'."
            )

    def camera_callback(self, camera_name: str, msg: Detection2DArray) -> None:
        context = self.camera_contexts[camera_name]
        if not context.calibration_ready:
            self.get_logger().warn(
                f"Ignoring '{camera_name}' detections: CameraInfo not initialized"
            )
            return

        stamp_s = stamp_to_seconds(msg.header.stamp)
        if context.duplicate_filter.seen(msg.header.stamp):
            self.get_logger().warn(
                f"Ignoring duplicate '{camera_name}' detection stamp "
                f"{stamp_key(msg.header.stamp)}"
            )
            return
        if stamp_s <= context.last_processed_stamp_s:
            self.get_logger().warn(
                f"Ignoring out-of-order '{camera_name}' detections: "
                f"{stamp_s:.9f} <= {context.last_processed_stamp_s:.9f}"
            )
            return

        source_frame = context.optical_frame_id or context.configured_frame_id
        if msg.header.frame_id and msg.header.frame_id != source_frame:
            self.get_logger().warn(
                f"'{camera_name}' Detection2DArray frame '{msg.header.frame_id}' "
                f"differs from CameraInfo frame '{source_frame}'; using CameraInfo frame."
            )

        transform = self._lookup_transform_to_map(
            source_frame, msg.header.stamp)
        if transform is None:
            return

        observations: list[BearingObservation] = []
        for detection in msg.detections:
            class_id, confidence = self._highest_confidence_class(detection)
            if class_id is None:
                continue

            config = self.class_configs.get(
                class_id, self.default_class_config)
            _, _, bbox_width, bbox_height = CameraContext._bbox_center_and_size(
                detection
            )
            if confidence < config.min_detector_confidence:
                continue
            if (
                bbox_width < config.min_bbox_width_px
                or bbox_height < config.min_bbox_height_px
            ):
                continue

            observation = context.detection_to_bearing(
                detection=detection,
                class_id=class_id,
                confidence=confidence,
                stamp=msg.header.stamp,
                transform=transform,
            )
            if observation is not None:
                observations.append(observation)

        assignments, promoted_tracks = self.fusion.process_camera_observations(
            observations
        )
        context.last_processed_stamp_s = stamp_s
        self._publish_camera_debug(
            context, observations, assignments, msg.header.stamp)

        for track in promoted_tracks:
            self._publish_semantic_track(track, observation_stamp_s=stamp_s)

        # Preserve the Version 1 refresh behavior, but use message time and the
        # stable promoted label. Only tracks associated in this camera message
        # are candidates for refresh.
        refreshed_track_ids: set[int] = set()
        for _, track in assignments:
            if (
                track.state is TrackState.LABELED
                and track.track_id not in refreshed_track_ids
                and stamp_s - track.last_semantic_publish_stamp_s
                >= self.settings.promoted_track_publish_period_s
            ):
                self._publish_semantic_track(
                    track, observation_stamp_s=stamp_s)
                refreshed_track_ids.add(track.track_id)

        # Promotion removes tracks from the complete unlabeled snapshot even if
        # no sonar message arrives immediately afterward.
        if promoted_tracks and self.latest_sonar_stamp_msg is not None:
            self._publish_unlabeled_track_snapshot(self.latest_sonar_stamp_msg)

    # ------------------------------------------------------------------
    # Message conversion and TF
    # ------------------------------------------------------------------

    def _highest_confidence_class(
        self,
        detection: Detection2D,
    ) -> tuple[int | None, float]:
        best_class: int | None = None
        best_confidence = -1.0
        for result in detection.results:
            mapped_class = self.name_map.get(result.hypothesis.class_id)
            if mapped_class is None:
                continue
            score = float(result.hypothesis.score)
            if score > best_confidence:
                best_class = mapped_class
                best_confidence = score
        return best_class, best_confidence

    def _lookup_transform_to_map(self, source_frame: str, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame=self.settings.map_frame,
                source_frame=source_frame,
                time=Time(
                    seconds=int(stamp.sec),
                    nanoseconds=int(stamp.nanosec),
                ),
            )
        except Exception as exception:  # tf2 exception classes vary by ROS distro.
            self.get_logger().warn(
                f"Rejecting measurement: failed transform from '{source_frame}' "
                f"to '{self.settings.map_frame}' at {stamp_key(stamp)}: {exception}"
            )
            return None

    @staticmethod
    def _canonicalize_cloud(cloud: PointCloud2) -> PointCloud2:
        points = np.array(
            [
                (x, y, z)
                for x, y, z in pc2.read_points(
                    cloud,
                    field_names=("x", "y", "z"),
                    skip_nans=True,
                )
            ],
            dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4")],
        )
        fields = [
            PointField(
                name="x",
                offset=0,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="y",
                offset=4,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="z",
                offset=8,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]
        return pc2.create_cloud(cloud.header, fields, points)

    def _transform_cloud_to_map(self, msg: PointCloud2) -> PointCloud2 | None:
        if msg.header.frame_id == self.settings.map_frame:
            return msg

        transform = self._lookup_transform_to_map(
            msg.header.frame_id,
            msg.header.stamp,
        )
        if transform is None:
            return None

        canonical = self._canonicalize_cloud(msg)
        transformed = do_transform_cloud(canonical, transform)
        transformed.header.frame_id = self.settings.map_frame
        transformed.header.stamp = msg.header.stamp
        return transformed

    # ------------------------------------------------------------------
    # Output publication
    # ------------------------------------------------------------------

    def _publish_unlabeled_track_snapshot(self, stamp) -> None:
        points = []
        stamp_s = stamp_to_seconds(stamp)
        for track in sorted(
            self.track_manager.confirmed_unlabeled_tracks(),
            key=lambda candidate: candidate.track_id,
        ):
            points.append(
                (
                    float(track.position_xy[0]),
                    float(track.position_xy[1]),
                    float(track.display_z(stamp_s, self.settings)),
                )
            )

        header = Header()
        header.frame_id = self.settings.map_frame
        header.stamp = stamp
        message = pc2.create_cloud_xyz32(header, points)
        self.unlabeled_tracks_publisher.publish(message)

    def _publish_semantic_track(
        self,
        track: Track,
        observation_stamp_s: float,
    ) -> None:
        if track.promoted_label is None:
            return

        pose = PoseStamped()
        pose.header.frame_id = self.settings.map_frame
        pose.header.stamp = track.last_sonar_stamp_msg
        pose.pose.position.x = float(track.position_xy[0])
        pose.pose.position.y = float(track.position_xy[1])
        pose.pose.position.z = float(
            track.display_z(observation_stamp_s, self.settings)
        )
        pose.pose.orientation.w = 1.0

        request = AddSemanticObject.Request()
        request.ids = [int(track.promoted_label)]
        request.positions = [pose]
        self.semantic_client.call_async(request)
        track.last_semantic_publish_stamp_s = observation_stamp_s

    def _publish_camera_debug(
        self,
        context: CameraContext,
        observations: Sequence[BearingObservation],
        assignments: Sequence[tuple[BearingObservation, Track]],
        stamp,
    ) -> None:
        if context.debug_publisher is None:
            return

        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.header.frame_id = self.settings.map_frame
        delete_marker.header.stamp = stamp
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        assigned_ids = {id(observation) for observation, _ in assignments}
        marker_id = 0
        for observation in observations:
            color = self._debug_color(observation.class_id)
            if id(observation) not in assigned_ids:
                color.a = 0.25

            for namespace, direction, width in (
                ("bearing_center", observation.center_direction_map, 0.035),
                ("bearing_left", observation.left_direction_map, 0.015),
                ("bearing_right", observation.right_direction_map, 0.015),
            ):
                marker = Marker()
                marker.header.frame_id = self.settings.map_frame
                marker.header.stamp = stamp
                marker.ns = f"{context.name}_{namespace}"
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.LINE_LIST
                marker.action = Marker.ADD
                marker.scale.x = width
                marker.color = color

                start = Point()
                start.x = float(observation.origin_map[0])
                start.y = float(observation.origin_map[1])
                start.z = float(observation.origin_map[2])

                end = Point()
                end.x = float(
                    observation.origin_map[0]
                    + self.settings.debug_ray_length_m * direction[0]
                )
                end.y = float(
                    observation.origin_map[1]
                    + self.settings.debug_ray_length_m * direction[1]
                )
                end.z = float(
                    observation.origin_map[2]
                    + self.settings.debug_ray_length_m * direction[2]
                )
                marker.points = [start, end]
                marker_array.markers.append(marker)

        context.debug_publisher.publish(marker_array)

    @staticmethod
    def _debug_color(class_id: int) -> ColorRGBA:
        if class_id == SemanticObject.SLALOM_RED:
            return ColorRGBA(r=1.0, a=1.0)
        if class_id == SemanticObject.SLALOM_WHITE:
            return ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        if class_id == SemanticObject.VERTICAL_MARKER:
            return ColorRGBA(a=1.0)
        return ColorRGBA(b=1.0, a=1.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImageCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
