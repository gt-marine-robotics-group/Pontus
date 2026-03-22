# from pontus_msgs.srv import AddSemanticObject
import geometry_msgs
from pontus_msgs.msg import SemanticObject
from pontus_msgs.srv import AddSemanticObject
from dataclasses import dataclass, field
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3Stamped, Vector3
from sensor_msgs.msg import PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_geometry_msgs

import numpy as np
from image_geometry import PinholeCameraModel
from math import acos, pi

from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2 as pc2

import tf_transformations
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2D, Detection2DArray

# ------ Track Dataclass -------
@dataclass
class CandidateTrack:
    track_id: int
    position_map: np.ndarray
    num_pc_detections: int = 1
    label_association_counts: dict[int, int] = field(default_factory=dict)
    total_label_associations: int = 0
    best_label: int | None = None
    best_label_count: int = 0
    first_seen_time_s: float = 0.0
    last_seen_time_s: float = 0.0
    last_publish_time_s: float = -1e9

    last_visual_range_m: float | None = None
    last_cluster_range_m: float | None = None
    last_line_error_m: float | None = None
    last_range_error_m: float | None = None
    last_assoc_score: float | None = None

    def update_position(self, point_xyz: np.ndarray) -> None:
        self.num_pc_detections += 1
        self.position_map += (point_xyz - self.position_map) / self.num_pc_detections

    def add_label_vote(self, class_id: int) -> None:
        new_count = self.label_association_counts.get(class_id, 0) + 1
        self.label_association_counts[class_id] = new_count
        self.total_label_associations += 1

        if new_count > self.best_label_count:
            self.best_label = class_id
            self.best_label_count = new_count


class ImageCoordinator(Node):
    def __init__(self) -> None:
        super().__init__('image_to_cluster_coordinator')

        self.latest_pointcloud: np.array = None
        # (m), width of line pointing toward object detected by yolo
        self.line_projection_width = .1
        # object score must be at least this to be added to semantic map
        self.confidence_min = 0.5
        self.vector_projection_dist = 10  # (m), how far the line is projected
        self.cam_model = PinholeCameraModel()
        self.camera_frame_name = 'camera_front'
        self.cam_initialized = False

        self.min_bbox_width = 0
        self.max_cluster_dist_m = 8.0

        # ------ Candidate tracking params ------
        self.cluster_track_match_dist_m = 0.30
        self.candidate_stale_time_s = 1.5
        self.pc_detection_threshold = 8
        self.label_assoc_threshold = 5
        self.promoted_track_publish_period_s = 0.75

        self.candidate_tracks: dict[int, CandidateTrack] = {}
        self.next_track_id = 0

        self.name_map = {
            'gate_side': SemanticObject.GATE_LEFT,
            'red_slalom': SemanticObject.SLALOM_RED,
            'reef_shark': SemanticObject.GATE_IMAGE_SHARK,
            'saw_shark': SemanticObject.GATE_IMAGE_FISH,
            'white_slalom': SemanticObject.SLALOM_WHITE,
            'path_marker': SemanticObject.PATH_MARKER,
            'vertical pole': SemanticObject.VERTICAL_MARKER,

            # Name from the sim yolo model is different from real apparently
            'slalom_red': SemanticObject.SLALOM_RED,
            'slalom_white': SemanticObject.SLALOM_WHITE,
            'gate_shark': SemanticObject.GATE_IMAGE_SHARK,
            'gate_fish': SemanticObject.GATE_IMAGE_FISH,
            'left_gate': SemanticObject.GATE_LEFT,
            'right_gate': SemanticObject.GATE_LEFT,
            'vertical_marker': SemanticObject.VERTICAL_MARKER,

            # Names from simulator bounding box cameras have to be integers so we also map the
            # label numbers
            str(SemanticObject.SLALOM_RED): SemanticObject.SLALOM_RED,
            str(SemanticObject.SLALOM_WHITE): SemanticObject.SLALOM_WHITE,
            str(SemanticObject.GATE_IMAGE_SHARK): SemanticObject.GATE_IMAGE_SHARK,
            str(SemanticObject.GATE_IMAGE_FISH): SemanticObject.GATE_IMAGE_FISH,
            str(SemanticObject.GATE_LEFT): SemanticObject.GATE_LEFT,
            # we currently don't distinguish between gate sides
            str(SemanticObject.GATE_RIGHT): SemanticObject.GATE_LEFT,
            str(SemanticObject.VERTICAL_MARKER): SemanticObject.VERTICAL_MARKER,
            str(SemanticObject.BIN): SemanticObject.BIN,
            str(SemanticObject.OCTAGON): SemanticObject.OCTAGON,
            str(SemanticObject.TARGET): SemanticObject.TARGET,
            str(SemanticObject.PATH_MARKER): SemanticObject.PATH_MARKER
        }


        # ------ Association tuning ------
        self.class_assoc_cfg = {
            SemanticObject.GATE_LEFT: {
                "min_bbox_px": 10.0,
                "line_threshold_m": 0.25,
                "range_threshold_m": 1.75,
                "range_weight": 0.75,
                "line_weight": 1.00,
                "size_prior_m": {"width": 0.0762},
            },
            SemanticObject.SLALOM_RED: {
                "min_bbox_px": 8.0,
                "line_threshold_m": 0.18,
                "range_threshold_m": 0.85,
                "range_weight": 1.00,
                "line_weight": 1.10,
                "size_prior_m": {"height": 0.90, "width": 0.0254},
            },
            SemanticObject.SLALOM_WHITE: {
                "min_bbox_px": 8.0,
                "line_threshold_m": 0.18,
                "range_threshold_m": 0.85,
                "range_weight": 1.00,
                "line_weight": 1.10,
                "size_prior_m": {"height": 0.90, "width": 0.0254},
            },
            SemanticObject.GATE_IMAGE_SHARK: {
                "min_bbox_px": 12.0,
                "line_threshold_m": 0.22,
                "range_threshold_m": 1.00,
                "range_weight": 1.20,
                "line_weight": 0.90,
                "size_prior_m": {"height": 0.305, "width": 0.305},
            },
            SemanticObject.GATE_IMAGE_FISH: {
                "min_bbox_px": 12.0,
                "line_threshold_m": 0.22,
                "range_threshold_m": 1.00,
                "range_weight": 1.20,
                "line_weight": 0.90,
                "size_prior_m": {"height": 0.305, "width": 0.305},
            },
            SemanticObject.PATH_MARKER: {
                "min_bbox_px": 10.0,
                "line_threshold_m": 0.35,
                "range_threshold_m": 1.25,
                "range_weight": 1.10,
                "line_weight": 0.90,
                "size_prior_m": {"height": 0.15, "width": 1.20},
            },
            SemanticObject.VERTICAL_MARKER: {
                "min_bbox_px": 8.0,
                "line_threshold_m": 0.25,
                "range_threshold_m": 1.00,
                "range_weight": 1.00,
                "line_weight": 1.00,
                "size_prior_m": {"height": 1.9812, "width": 0.0508},
            },
            SemanticObject.BIN: {
                "min_bbox_px": 12.0,
                "line_threshold_m": 0.40,
                "range_threshold_m": 1.50,
                "range_weight": 1.10,
                "line_weight": 0.90,
                "size_prior_m": {"height": 0.305, "width": 0.610},
            },
            SemanticObject.OCTAGON: {
                "min_bbox_px": 20.0,
                "line_threshold_m": 0.60,
                "range_threshold_m": 2.00,
                "range_weight": 0.80,
                "line_weight": 0.70,
                "size_prior_m": {"height": 2.70, "width": 2.70},
            },
            SemanticObject.TARGET: {
                "min_bbox_px": 12.0,
                "line_threshold_m": 0.30,
                "range_threshold_m": 1.20,
                "range_weight": 1.00,
                "line_weight": 1.00,
                "size_prior_m": {"height": 0.60, "width": 0.60},
            },
        }

        self.default_assoc_cfg = {
            "min_bbox_px": 8.0,
            "line_threshold_m": self.line_projection_width,
            "range_threshold_m": None,
            "range_weight": 0.0,
            "line_weight": 1.0,
            "size_prior_m": {},
        }


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cli = self.create_client(
            AddSemanticObject, '/pontus/add_semantic_object')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.cluster = self.create_subscription(
            PointCloud2,
            '/pontus/sonar/clustercloud',
            self.pointcloud_callback,
            10
        )

        self.yolo = self.create_subscription(
            Detection2DArray,
            '/pontus/{}/yolo_results'.format(self.camera_frame_name),
            self.yolo_callback,
            10
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            # Replace with your camera info topic
            '/pontus/{}/camera_info'.format(self.camera_frame_name),
            self.info_callback,
            10
        )

        self.debug_line_pub = self.create_publisher(
            MarkerArray,
            '/pontus/{}/debug_lines'.format(self.camera_frame_name),
            10
        )
        self.debug_id = 0

    def yolo_callback(self, msg: Detection2DArray) -> None:
        """Assigns results from yolo to points in latest_pointcloud and publishes all points as list

        Args:
            msg (Detection2DArray): _description_
        """
        if not self.cam_initialized:
            self.get_logger().warn("camera info not initialized")
            return
        elif self.latest_pointcloud is None:
            self.get_logger().warn("No point cloud found")
            return

        # TODO: This should be removed when not testing on old bag data
        msg.header.frame_id = "camera_front_optical_frame"

        array_msg = MarkerArray()
        marker_msg = Marker()
        marker_msg.header = msg.header
        marker_msg.ns = "lines"
        marker_msg.action = Marker.DELETEALL
        array_msg.markers.append(marker_msg)
        self.debug_line_pub.publish(array_msg)

        for detection in msg.detections:
            # Keep the Humble bag-data workaround, but apply it to the individual detection too.
            detection.header.frame_id = "camera_front_optical_frame"

            if detection.bbox.size_x < self.min_bbox_width:
                continue

            class_id, confidence = self.get_highest_confidence_class(detection)
            if class_id is None or confidence < self.confidence_min:
                continue

            assoc_cfg = self.get_assoc_cfg(class_id)
            if max(detection.bbox.size_x, detection.bbox.size_y) < assoc_cfg["min_bbox_px"]:
                continue

            matched_track, object_pose = self.associate_object_with_track(
                detection, class_id)

            if matched_track is None or object_pose is None:
                continue

            matched_track.add_label_vote(class_id)

            if self.track_ready_for_promotion(matched_track):
                if matched_track.best_label is None or matched_track.best_label != class_id:
                    continue

                now_s = self._now_s()
                if (now_s - matched_track.last_publish_time_s) >= self.promoted_track_publish_period_s:
                    matched_track.last_publish_time_s = now_s
                    self.send_request(
                        [matched_track.best_label], [object_pose])


    def pointcloud_callback(self, msg: PointCloud2) -> None:
        """
        Saves recieved pointcloud to latest_pointcloud, taken from occupency_grid_manager.py
        Args:
        ----
        msg (PointCloud2): subscribed pointcloud
        """

        # clustered_cloud is already in correct space, optionally add transform logic here just in case
        if msg.header.frame_id != 'map':
            transformed_msg = self.transform_sonar(msg)
        else:
            transformed_msg = msg

        points = point_cloud2.read_points_numpy(
            transformed_msg, field_names=('x', 'y', 'z'),
            skip_nans=True
        )
        # points[:, 1] = -points[:, 1]  # TODO ask why do this?

        self.latest_pointcloud = points
        self.update_candidate_tracks(points)

    def info_callback(self, msg):
        """
        Callback for CameraInfo messages. Updates the PinholeCameraModel.
        """
        if not self.cam_initialized:  # Only update if not initialized or if parameters change
            self.cam_model.fromCameraInfo(msg)
            # self.get_logger().warn(f"cam_model: {self.cam_model}")
            self.cam_initialized = True
            self.get_logger().info('Camera model initialized with new info.')

    def get_highest_confidence_class(self, object_msg: Detection2D) -> tuple[int | None, float]:
        max_confidence = -1.0
        highest_class: int | None = None

        for result in object_msg.results:
            mapped_class = self.name_map.get(result.hypothesis.class_id)
            if mapped_class is None:
                continue

            if result.hypothesis.score > max_confidence:
                max_confidence = result.hypothesis.score
                highest_class = mapped_class

        return highest_class, max_confidence

    def update_candidate_tracks(self, point_array: np.ndarray) -> None:
        self.prune_stale_tracks()

        if point_array.size == 0:
            return

        now_s = self._now_s()
        updated_track_ids: set[int] = set()

        for point in point_array:
            point_xyz = np.asarray(point, dtype=float)

            matched_track = None
            matched_dist = float('inf')

            for track in self.candidate_tracks.values():
                if track.track_id in updated_track_ids:
                    continue

                dist_xy = np.linalg.norm(
                    track.position_map[:2] - point_xyz[:2])

                if dist_xy <= self.cluster_track_match_dist_m and dist_xy < matched_dist:
                    matched_track = track
                    matched_dist = dist_xy

            if matched_track is None:
                self.candidate_tracks[self.next_track_id] = CandidateTrack(
                    track_id=self.next_track_id,
                    position_map=point_xyz.copy(),
                    first_seen_time_s=now_s,
                    last_seen_time_s=now_s,
                )

                updated_track_ids.add(self.next_track_id)
                self.next_track_id += 1
                continue

            matched_track.update_position(point_xyz)
            matched_track.last_seen_time_s = now_s
            updated_track_ids.add(matched_track.track_id)

    def prune_stale_tracks(self) -> None:
        now_s = self._now_s()
        stale_track_ids = [
            track_id
            for track_id, track in self.candidate_tracks.items()
            if (now_s - track.last_seen_time_s) > self.candidate_stale_time_s
        ]

        for track_id in stale_track_ids:
            del self.candidate_tracks[track_id]

    def track_ready_for_promotion(self, track: CandidateTrack) -> bool:
        pc_detection_test = track.num_pc_detections >= self.pc_detection_threshold
        label_detection_test = track.best_label_count >= self.label_assoc_threshold
        return pc_detection_test and label_detection_test

    def get_assoc_cfg(self, class_id: int) -> dict:
        return self.class_assoc_cfg.get(class_id, self.default_assoc_cfg)

    def estimate_range_from_bbox(self, object_msg: Detection2D, class_id: int) -> float | None:
        """
        Estimate monocular range from bbox size using known object dimensions.
        Returns None if no reliable prior exists for the class.
        """
        cfg = self.get_assoc_cfg(class_id)
        size_prior = cfg["size_prior_m"]

        estimates = []

        bbox_w_px = float(object_msg.bbox.size_x)
        bbox_h_px = float(object_msg.bbox.size_y)

        try:
            fx = float(self.cam_model.fx())
            fy = float(self.cam_model.fy())
        except Exception:
            return None

        if "width" in size_prior and bbox_w_px > 0:
            estimates.append((size_prior["width"] * fx) / bbox_w_px)

        if "height" in size_prior and bbox_h_px > 0:
            estimates.append((size_prior["height"] * fy) / bbox_h_px)

        if not estimates:
            return None

        return float(np.median(estimates))

    def build_camera_ray_pose(self, object_msg: Detection2D) -> PoseStamped:
        """
        Humble-safe version:
        - keep fromCameraInfo / rectifyPoint / projectPixelTo3dRay API
        - keep the camera frame override you needed on the robot
        """
        camera_pose = PoseStamped()
        camera_pose.header = object_msg.header
        camera_pose.header.frame_id = "camera_front_optical_frame"

        center_position = object_msg.bbox.center.position

        point = np.array([center_position.x, center_position.y], dtype=np.float64)
        rectified_point = self.cam_model.rectifyPoint(point)
        ray_unit_vector = self.cam_model.projectPixelTo3dRay(rectified_point)

        pose_to_object = self.align_pose_x_with_vector(
            camera_pose, ray_unit_vector)
        transformed_pose = self.transform_camera(pose_to_object)

        return transformed_pose

    def make_object_pose_from_track(
        self,
        transformed_pose: PoseStamped,
        track: CandidateTrack
    ) -> PoseStamped:
        selected_point = Point()
        selected_point.x = float(track.position_map[0])
        selected_point.y = float(track.position_map[1])
        selected_point.z = float(track.position_map[2])

        # Get the camera ray direction in map frame
        vector_to_object = self.get_x_axis_vector_from_pose(transformed_pose)

        cam_pos = np.array([
            transformed_pose.pose.position.x,
            transformed_pose.pose.position.y,
            transformed_pose.pose.position.z
        ], dtype=float)

        dir_vec = np.array([
            vector_to_object.vector.x,
            vector_to_object.vector.y,
            vector_to_object.vector.z
        ], dtype=float)

        dir_xy = dir_vec[:2]
        target_xy = np.array([selected_point.x, selected_point.y], dtype=float)

        denom = float(dir_xy.dot(dir_xy))
        if denom > 1e-8:
            t = dir_xy.dot(target_xy - cam_pos[:2]) / denom
            if t < 0.0:
                t = 0.0

            z_est = cam_pos[2] + t * dir_vec[2]
            selected_point.z = float(z_est)

        object_pose = PoseStamped()
        object_pose.header = transformed_pose.header
        object_pose.pose.position = selected_point

        return object_pose

    def score_track_for_detection(
        self,
        transformed_pose: PoseStamped,
        track: CandidateTrack,
        class_id: int,
        visual_range_m: float | None
    ) -> tuple[float, float, float, float] | None:
        """
        Returns (score, line_dist_m, cluster_range_m, range_err_m) if valid, else None.
        """
        cfg = self.get_assoc_cfg(class_id)

        vector_to_object = self.get_x_axis_vector_from_pose(transformed_pose)

        start_xy = np.array([
            transformed_pose.pose.position.x,
            transformed_pose.pose.position.y
        ], dtype=float)

        dir_xy = np.array([
            vector_to_object.vector.x,
            vector_to_object.vector.y
        ], dtype=float)

        dir_xy_norm = np.linalg.norm(dir_xy)
        if dir_xy_norm < 1e-8:
            return None

        dir_xy = dir_xy / dir_xy_norm

        delta_xy = track.position_map[:2] - start_xy
        along_track_dist = float(delta_xy.dot(dir_xy))

        if along_track_dist < 0.0 or along_track_dist > self.max_cluster_dist_m:
            return None

        line_dist = abs(delta_xy[0] * dir_xy[1] - delta_xy[1] * dir_xy[0])
        if line_dist > cfg["line_threshold_m"]:
            return None

        cam_pos = np.array([
            transformed_pose.pose.position.x,
            transformed_pose.pose.position.y,
            transformed_pose.pose.position.z
        ], dtype=float)

        cluster_range_m = float(np.linalg.norm(track.position_map - cam_pos))

        range_err_m = 0.0
        if visual_range_m is not None and cfg["range_threshold_m"] is not None:
            range_err_m = abs(cluster_range_m - visual_range_m)
            if range_err_m > cfg["range_threshold_m"]:
                return None

        line_term = cfg["line_weight"] * (
            line_dist / max(cfg["line_threshold_m"], 1e-6)
        )

        range_term = 0.0
        if visual_range_m is not None and cfg["range_threshold_m"] is not None:
            range_term = cfg["range_weight"] * (
                range_err_m / max(cfg["range_threshold_m"], 1e-6)
            )

        persistence_bonus = 0.05 * min(track.num_pc_detections,
                                       self.pc_detection_threshold)
        range_bias = 0.02 * cluster_range_m

        score = line_term + range_term + range_bias - persistence_bonus
        return score, line_dist, cluster_range_m, range_err_m

    def find_best_track_for_detection(
        self,
        transformed_pose: PoseStamped,
        class_id: int,
        visual_range_m: float | None
    ) -> CandidateTrack | None:
        if not self.candidate_tracks:
            return None

        best_track = None
        best_score = float('inf')

        for track in self.candidate_tracks.values():
            scored = self.score_track_for_detection(
                transformed_pose=transformed_pose,
                track=track,
                class_id=class_id,
                visual_range_m=visual_range_m,
            )
            if scored is None:
                continue

            score, line_dist, cluster_range_m, range_err_m = scored

            if score < best_score:
                best_track = track
                best_score = score

                track.last_visual_range_m = visual_range_m
                track.last_cluster_range_m = cluster_range_m
                track.last_line_error_m = line_dist
                track.last_range_error_m = range_err_m if visual_range_m is not None else None
                track.last_assoc_score = score

        return best_track

    def associate_object_with_track(
        self,
        object_msg: Detection2D,
        class_id: int | None = None
    ) -> tuple[CandidateTrack | None, PoseStamped | None]:
        if not self.candidate_tracks:
            return None, None

        if class_id is None:
            class_id, _ = self.get_highest_confidence_class(object_msg)
            if class_id is None:
                return None, None

        transformed_pose = self.build_camera_ray_pose(object_msg)

        array_msg = MarkerArray()
        marker_msg = Marker()
        marker_msg.header = transformed_pose.header
        marker_msg.pose = transformed_pose.pose
        marker_msg.ns = "lines"
        marker_msg.id = self.debug_id
        self.debug_id += 1
        marker_msg.type = Marker.ARROW
        marker_msg.action = Marker.ADD
        marker_msg.frame_locked = True
        marker_msg.scale.x = 15.0
        marker_msg.scale.y = 0.05
        marker_msg.scale.z = 0.05
        marker_msg.color = self.get_debug_line_color(class_id)
        array_msg.markers.append(marker_msg)
        self.debug_line_pub.publish(array_msg)

        visual_range_m = self.estimate_range_from_bbox(object_msg, class_id)

        matched_track = self.find_best_track_for_detection(
            transformed_pose=transformed_pose,
            class_id=class_id,
            visual_range_m=visual_range_m,
        )
        if matched_track is None:
            return None, None

        object_pose = self.make_object_pose_from_track(transformed_pose, matched_track)
        return matched_track, object_pose


    def align_pose_x_with_vector(self, pose_msg: PoseStamped, target_vector: list) -> PoseStamped:
        """
        Aligns the x-axis of a given Pose orientation with a target 3D vector.

        Args
            :param pose_msg: The original geometry_msgs.msg.PoseStamped message.
            :param target_vector: The desired direction as a list unit vector.
        Returns
            :return: A new Pose message with the updated orientation.
        """
        # 1. Define the initial X-axis vector (1, 0, 0)
        initial_x_axis = np.array([1.0, 0.0, 0.0])

        target_array = np.array(target_vector)

        # 2. Calculate the rotation axis and angle
        # The axis of rotation is the cross product of the initial and target vectors
        rotation_axis = np.cross(initial_x_axis, target_array)
        rotation_axis_norm = np.linalg.norm(rotation_axis)

        if rotation_axis_norm == 0:
            # Vectors are parallel (either same or opposite direction)
            if np.dot(initial_x_axis, target_array) > 0:
                # Same direction, no rotation needed
                return pose_msg
            else:
                # Opposite direction (180 degrees rotation around any orthogonal axis, e.g., Y-axis)
                rotation_axis = np.array([0.0, 1.0, 0.0])
                angle = pi
        else:
            # Normalize the rotation axis
            rotation_axis = rotation_axis / rotation_axis_norm
            # The angle of rotation can be found using the dot product (angle = acos(dot(u, v)))
            angle = acos(np.dot(initial_x_axis, target_array))

        # 3. Convert the axis-angle representation to a quaternion
        quat_new = tf_transformations.quaternion_about_axis(
            angle, rotation_axis)

        # 4. Update the Pose message
        new_pose = PoseStamped()
        new_pose.header = pose_msg.header
        # Keep the original position, should be all zeros
        new_pose.pose.position = pose_msg.pose.position
        new_pose.pose.orientation.x = quat_new[0]
        new_pose.pose.orientation.y = quat_new[1]
        new_pose.pose.orientation.z = quat_new[2]
        new_pose.pose.orientation.w = quat_new[3]

        return new_pose

    def get_x_axis_vector_from_pose(self, pose_msg: PoseStamped) -> Vector3Stamped:
        """
        Extracts the x-axis direction vector from a geometry_msgs/Pose message.
        """
        # 1. Define the unit vector along the local x-axis

        # local_x_axis_vector = Vector3Stamped(x=1.0, y=0.0, z=0.0)

        local_x_axis_vector = Vector3Stamped()
        local_x_axis_vector.header = pose_msg.header
        local_x_axis_vector.vector = Vector3(x=1.0, y=0.0, z=0.0)

        # 2. Create a transform using only the orientation of the pose
        # We use a dummy transform with zero translation for this
        transform = geometry_msgs.msg.Transform(
            translation=Vector3(x=0.0, y=0.0, z=0.0),
            rotation=pose_msg.pose.orientation
        )

        # 3. Create a TransformStamped message for the tf2 utility
        transform_stamped = geometry_msgs.msg.TransformStamped()
        transform_stamped.transform = transform

        # 4. Transform the local x-axis vector to the pose's frame of reference
        # do_transform_vector3 applies the rotation defined by the transform
        global_x_axis_vector = tf2_geometry_msgs.do_transform_vector3(
            local_x_axis_vector, transform_stamped)

        return global_x_axis_vector

    def points_on_line(self, point_array: np.ndarray, pose: PoseStamped) -> np.ndarray:
        """
        Checks that provided point exists on line defined by pose, this could be improved

        Args:
            point_array (Point): array of points to be checked against
            pose (Pose): pose that defines line of interest

        Returns:
            np.ndarray: structured numpy array of all points on the line defined by pose
        """

        point2_array = point_array.copy()
        point2_array[:, 2] = 0

        vector_to_object = self.get_x_axis_vector_from_pose(pose)

        start_point = np.array(
            [pose.pose.position.x, pose.pose.position.y, 0.0], dtype=point_array.dtype)

        # Range filtering
        ranges = np.linalg.norm(point2_array - start_point, axis=1)
        in_range = ranges <= self.max_cluster_dist_m

        point_array = point_array[in_range]
        point2_array = point2_array[in_range]

        if point_array.size == 0:
            return point_array.reshape(0, point_array.shape[1])

        end_point = np.array([vector_to_object.vector.x*self.vector_projection_dist + pose.pose.position.x,
                              vector_to_object.vector.y*self.vector_projection_dist + pose.pose.position.y,
                              0.0], dtype=point_array.dtype)

        ba = start_point - end_point
        bc = point2_array - end_point

        distance = np.sum(np.abs(np.cross(ba, bc)), axis=1) / \
            (np.sum(np.abs(ba)) + 0.000000001)

        keys = np.argsort(distance)
        sorted_point_array = point_array[keys]
        sorted_distance_array = np.sort(distance)

        return sorted_point_array[sorted_distance_array <= self.line_projection_width, :]

    @staticmethod
    def _canonicalize_cloud(cloud: PointCloud2) -> PointCloud2:
        pts = np.array([(x, y, z) for x, y, z in pc2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True)],
                       dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
        fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]
        new_cloud = pc2.create_cloud(cloud.header, fields, pts)
        return new_cloud

    def transform_sonar(self, msg: PointCloud2) -> PointCloud2:
        """
        transforms the frame from sonar to map

        Args:
        ----
        msg (PointCloud2): subscribed pointcloud

        Return:
        ----
        (PointCloud2): The pointedcloud with transformed data
        """
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame=msg.header.frame_id,
                time=Time(seconds=msg.header.stamp.sec,
                          nanoseconds=msg.header.stamp.nanosec)
            )
            # self.get_logger().info("SUCCESS ON TRANSFORM")
        except Exception as e:
            self.get_logger().warn(
                "failure to transform pointcloud to map frame, current frame: {}".format(msg.header.frame_id))
            self.get_logger().warn(f"exception: {e}")
            return msg

        msg_canon = self._canonicalize_cloud(msg)
        transformed_msg = do_transform_cloud(msg_canon, transform)

        return transformed_msg

    def transform_camera(self, msg: PoseStamped) -> PoseStamped:
        """
        transforms the frame from sonar to map

        Args:
        ----
        msg (PoseStamped): posestamped in camera frame

        Return:
        ----
        (PoseStamped): The posestamped with transformed data
        """
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame=msg.header.frame_id,
                time=Time(seconds=msg.header.stamp.sec,
                          nanoseconds=msg.header.stamp.nanosec)
            )
            pose_transformed = tf2_geometry_msgs.do_transform_pose(
                msg.pose, transform)
            # self.get_logger().info("SUCCESS ON TRANSFORM")
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header = msg.header
            pose_stamped_msg.header.frame_id = "map"
            pose_stamped_msg.pose = pose_transformed
            return pose_stamped_msg
        except Exception as e:
            self.get_logger().warn(
                "failure to transform pose to map frame, current frame: {}".format(msg.header.frame_id))
            self.get_logger().warn(f"exception: {e}")
            return msg

    def send_request(self, class_id, pose):
        req = AddSemanticObject.Request()
        req.ids = class_id
        req.positions = pose
        self.cli.call_async(req)

    def get_debug_line_color(self, class_id):
        match class_id:
            case SemanticObject.SLALOM_RED:
                return ColorRGBA(r=1.0, a=1.0)
            case SemanticObject.SLALOM_WHITE:
                return ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            case SemanticObject.VERTICAL_MARKER:
                return ColorRGBA(a=1.0)
            case default:
                return ColorRGBA(b=1.0, a=1.0)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = ImageCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
