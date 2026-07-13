"""
Semantic Map Manager

- Maintains a SemanticMap message grouped by object type
- Fuses duplicate detections with an online mean of positions
- Publishes a MarkerArray visualization
- Exposes a service to add semantic objects
"""

# ------ Libraries ------
from dataclasses import dataclass, field
from typing import Iterator, Optional, List
import math
import numpy as np
import sys
from enum import Enum
import sensor_msgs_py.point_cloud2 as pc2

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

import tf2_ros
import tf2_geometry_msgs

from std_msgs.msg import Header, String
from geometry_msgs.msg import Pose, PoseWithCovariance, Point, Quaternion, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2

from pontus_msgs.msg import SemanticObject, SemanticMap, SemanticMetaGate, SemanticMetaSlalomRow, SemanticMetaSlalom
from pontus_msgs.srv import AddSemanticObject, AddMetaGate
from pontus_tools.transform_helpers import convert_to_map_frame


# ------- Semantic Map Wrapper ------
@dataclass
class SemanticMapDC:
    """Lightweight manager around the SemanticMap ROS message"""

    semantic_map: SemanticMap = field(default_factory=SemanticMap)
    objects: dict[int, list[SemanticObject]] = field(init=False)

    # ------ Meta Objects ------
    meta_gate: Optional[SemanticMetaGate] = None
    meta_slalom: Optional[SemanticMetaSlalom] = None

    def __post_init__(self):
        self.objects = {
            SemanticObject.GATE_IMAGE_SHARK: self.semantic_map.gate_image_shark,
            SemanticObject.GATE_IMAGE_FISH:  self.semantic_map.gate_image_fish,
            SemanticObject.GATE_LEFT:        self.semantic_map.gate_left,
            SemanticObject.GATE_RIGHT:       self.semantic_map.gate_right,
            SemanticObject.SLALOM_RED:       self.semantic_map.slalom_red,
            SemanticObject.SLALOM_WHITE:     self.semantic_map.slalom_white,
            SemanticObject.VERTICAL_MARKER:  self.semantic_map.vertical_marker,
            SemanticObject.BIN:              self.semantic_map.bin,
            SemanticObject.OCTAGON:          self.semantic_map.octagon,
            SemanticObject.TARGET:           self.semantic_map.target,
        }

        if self.meta_gate is not None:
            self.semantic_map.meta_gate = self.meta_gate

        if self.meta_slalom:
            self.semantic_map.meta_slalom = self.meta_slalom

    @staticmethod
    def check_semantic_object_duplicant(obj1: SemanticObject, obj2: SemanticObject) -> bool:
        """Return True if obj1 and obj2 are same type and within duplicate tolerance."""
        if not isinstance(obj1, SemanticObject) or not isinstance(obj2, SemanticObject):
            return False

        if obj1.object_type != obj2.object_type:
            return False

        p1 = obj1.pose.pose.position
        p2 = obj2.pose.pose.position

        dist = np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
        # dist = np.linalg.norm(p1 - p2)

        duplicant_tolerance_m = min(
            obj1.duplicant_tolerance_m, obj2.duplicant_tolerance_m)
        return dist <= duplicant_tolerance_m

    def add(self, obj: SemanticObject) -> None:
        """Add or fuse a semantic object into the map."""
        obj_list = self.objects.get(int(obj.object_type))
        if obj_list is None:
            return

        # Only add valid vertical markers based on the gate location
        if obj.object_type == SemanticObject.VERTICAL_MARKER:
            if not self.should_add_vertical_marker(obj):
                return
            # self.get_logger().info("Adding Vertical Marker")

        for existing in obj_list:
            if self.check_semantic_object_duplicant(obj, existing):
                # Fuse via online mean for position only.
                existing.num_detections += 1
                n = existing.num_detections

                p_old = existing.pose.pose.position
                p_new = obj.pose.pose.position

                # Online mean: mean += (new - mean) / n
                p_old.x += (p_new.x - p_old.x) / n
                p_old.y += (p_new.y - p_old.y) / n
                p_old.z += (p_new.z - p_old.z) / n

                # TODO: confidence update policy
                # existing.confidence = max(existing.confidence, obj.confidence)

                # Timestamps: keep latest
                existing.header.stamp = obj.header.stamp
                existing.last_updated = obj.last_updated
                return

        obj_list.append(obj)

    def remove(self, obj: SemanticObject) -> None:
        label_list = self.objects[obj.object_type]
        label_list.remove(obj)

    # ------ Add Meta Objects ------
    def add_meta_gate(self, left_gate: SemanticObject, right_gate: SemanticObject) -> None:
        # self.get_logger().info("Adding Meta Gate")
        gate = SemanticMetaGate()

        gate.header = Header()
        gate.header.stamp = left_gate.header.stamp
        gate.header.frame_id = left_gate.header.frame_id

        gate.left_gate = left_gate
        gate.right_gate = right_gate

        self.meta_gate = gate
        self.semantic_map.meta_gate = self.meta_gate

    def add_meta_slalom(self, slalom_rows: List[SemanticMetaSlalomRow]) -> None:
        # self.get_logger().info("Adding Meta Slalom")
        slalom = SemanticMetaSlalom()

        slalom.header = Header()
        slalom.header.stamp = slalom_rows[0].slalom_red.header.stamp
        slalom.header.frame_id = slalom_rows[0].slalom_red.header.frame_id

        slalom.meta_slalom_rows = slalom_rows

        self.meta_slalom = slalom
        self.semantic_map.meta_slalom = self.meta_slalom
    
    def should_add_vertical_marker(self, obj):
        if self.meta_gate is None:
            return

        def pose_to_nparray(msg: Pose) -> np.ndarray:
            return np.array([
                msg.position.x,
                msg.position.y],
                dtype=float
            )
        g1: np.ndarray = pose_to_nparray(
            self.meta_gate.left_gate.pose.pose)
        g2: np.ndarray = pose_to_nparray(
            self.meta_gate.right_gate.pose.pose)

        # Gate vector from left side to right side
        gate_vec = g2 - g1
        gate_unit_vec = gate_vec / np.linalg.norm(gate_vec)

        # Perpendicular Vector
        perp_vec = np.array([-gate_vec[1], gate_vec[0]])
        perp_unit_vec = perp_vec / np.linalg.norm(perp_vec)

        midpoint = (g1 + g2) / 2.0

        marker_vec: np.ndarray = np.array([
            obj.pose.pose.position.x,
            obj.pose.pose.position.y],
            dtype=float
        )

        marker_gate_vec = marker_vec - midpoint

        parallel = np.dot(marker_gate_vec, gate_unit_vec) * gate_unit_vec
        # perp = marker_gate_vec - parallel
        # dist = float(np.linalg.norm(perp))
        angle = np.arcsin(np.dot(perp_vec, marker_gate_vec) / (np.linalg.norm(perp_vec) * np.linalg.norm(marker_gate_vec)))

        dist_from_gate = np.linalg.norm(marker_gate_vec)

        print(f"considering {angle} radians, {dist_from_gate} m", file=sys.stderr)

        return abs(angle) <= np.pi/6 and abs(dist_from_gate) >= 4.0

    def create_message(self) -> SemanticMap:
        """Return the SemanticMap ROS message"""
        return self.semantic_map

    def get_meta_objects(self) -> List[SemanticObject]:
        """
        TODO: Absolutley better way to do this I'm just in a rush and can't think straight
        """
        output = []

        if self.meta_gate:
            output.append(self.meta_gate.left_gate)
            output.append(self.meta_gate.right_gate)

        if self.meta_slalom:
            for row in self.meta_slalom.meta_slalom_rows:
                output.append(row.slalom_red)
                for white in row.slaloms_white:
                    output.append(white)

        return output

    def __iter__(self) -> Iterator[SemanticObject]:
        """Iterate over all semantic objects in the map"""
        for obj_list in self.objects.values():
            for obj in obj_list:
                yield obj

class CandidateKind(Enum):
    SLALOM_RED = 1
    SLALOM_WHITE = 2
    UNKNOWN_TRACK = 3

@dataclass
class SlalomCandidate:
    kind: CandidateKind
    pose: np.ndarray
    known: bool
    obj: Optional[SemanticObject] = None

    def build_slalom_candidates_from_semantic_objects(semantic_objs: List[SemanticObject]) -> List['SlalomCandidate']:
        candidates = []
        for semantic_obj in semantic_objs:
            if semantic_obj.object_type == SemanticObject.SLALOM_RED:
                type = CandidateKind.SLALOM_RED
            elif semantic_obj.object_type == SemanticObject.SLALOM_WHITE:
                type = CandidateKind.SLALOM_WHITE
            else:
                type = CandidateKind.UNKNOWN_TRACK

            pose = np.array([
                semantic_obj.pose.pose.position.x,
                semantic_obj.pose.pose.position.y
            ], dtype=float)

            candidates.append(SlalomCandidate(kind=type, pose=pose, known=True, obj=semantic_obj))

        return candidates
    
    def build_slalom_candidates_from_sonar_tracks(sonar_tracks: List[np.ndarray]) -> List['SlalomCandidate']:
        candidates = []
        for track in sonar_tracks:
            candidates.append(SlalomCandidate(kind=CandidateKind.UNKNOWN_TRACK, pose=track, known=False))
        return candidates
    
@dataclass
class SlalomRowProposal:
    red_pos: np.ndarray
    line_unit_vec: np.ndarray
    white1: SlalomCandidate
    red: SlalomCandidate
    white2_pos: np.ndarray

# ------- Node ------
class SemanticMapManager(Node):

    def __init__(self):

        super().__init__('semantic_map_manager')

        # ------ Parameters ------
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gate_width', 3.048),          # RoboSub handbook
                ('gate_width_tolerance', 0.5),  # tolerance for pairing
                ("slalom_white_to_white_width", 3.05),
                ("slalom_white_to_red_width", 1.52),
                ("slalom_width_tolerance", 0.5),  # slalom pairing tolerance
                # red slalom row deviation tolerance
                ("slalom_row_width", 2.0),
                ("slalom_row_tolerance", 0.4),
                ("slalom_dup_tolerance", 0.5),
                ("slalom_row_deviation", 1.2), # max deviation along row direction between rows
                ("slalom_row_deviation_deg", 15.0), # max deviation along row direction between rows
            ]
        )

        self.gate_width_m = float(self.get_parameter('gate_width').value)
        self.gate_width_tolerance_m = float(
            self.get_parameter('gate_width_tolerance').value)

        self.slalom_white_to_white_width = float(
            self.get_parameter('slalom_white_to_white_width').value)
        self.slalom_white_to_red_width = float(
            self.get_parameter('slalom_white_to_red_width').value)
        self.slalom_width_tolerance = float(
            self.get_parameter("slalom_width_tolerance").value)
        self.slalom_row_width = float(self.get_parameter("slalom_row_width").value)
        self.slalom_row_tolerance = float(
            self.get_parameter("slalom_row_tolerance").value)
        self.slalom_dup_tolerance = float(
            self.get_parameter("slalom_dup_tolerance").value)
        self.slalom_row_deviation = float(
            self.get_parameter("slalom_row_deviation").value)
        self.slalom_row_deviation_deg = float(
            self.get_parameter("slalom_row_deviation_deg").value)
        
        self.sonar_tracks: List[SlalomCandidate] = []
        self.row_candidates = []

        # Map Services
        self.add_semantic_object_srv = self.create_service(
            AddSemanticObject,
            '/pontus/add_semantic_object',
            self.add_semantic_object_callback,
        )

        # Publishers
        self.semantic_map_pub = self.create_publisher(
            SemanticMap,
            '/pontus/semantic_map',
            10
        )

        self.semantic_map_visual_pub = self.create_publisher(
            MarkerArray,
            '/pontus/semantic_map_visual',
            10
        )

        self.semantic_map_debug_pub = self.create_publisher(
            String,
            '/pontus/semantic_map_debug',
            10
        )

        # Subscriptions
        self.create_subscription(
            PointCloud2,
            '/pontus/unlabeled_candidate_tracks',
            self.update_candidate_tracks,
        )

        map_visual_period = 1.0
        self.map_visual_timer = self.create_timer(
            map_visual_period, self.publish_semantic_map_visual)

        self.semantic_map = SemanticMapDC()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.meta_removal_threshold = 0.5

        # TODO: put semantic_map publisher on a timer
        self.create_timer(1.0, self.publish_semantic_map)

        self.create_timer(1.0, self._update_meta_slalom)

    def add_semantic_object_callback(self,
                                     request: AddSemanticObject.Request,
                                     response: AddSemanticObject.Response) -> AddSemanticObject.Response:
        """
        Add objects by transforming detections into 'map' and fusing into the semantic map.
        Expects request.ids and request.positions
        """

        now = self.get_clock().now().to_msg()

        for class_id, detection_pose in zip(request.ids, request.positions):
            # Convert to 'map' frame Pose (returns None if TF unavailable)
            map_pose: Optional[Pose] = convert_to_map_frame(
                detection_pose, self.tf_buffer, target_frame="map")

            if map_pose is None:
                self.get_logger().warn('Unable to find map transform, skipping add')
                continue

            # Build SemanticObject message
            obj = SemanticObject()
            obj.header = Header()
            obj.header.stamp = now
            obj.header.frame_id = "map"

            obj.object_type = int(class_id)

            obj.pose = PoseWithCovariance()
            obj.pose.pose.position = Point(
                x=map_pose.position.x,
                y=map_pose.position.y,
                z=map_pose.position.z,
            )
            obj.pose.pose.orientation = Quaternion(
                x=map_pose.orientation.x,
                y=map_pose.orientation.y,
                z=map_pose.orientation.z,
                w=map_pose.orientation.w,
            )

            obj.num_detections = 1
            obj.confidence = 1.0

            obj.last_updated = now
            obj.duplicant_tolerance_m = 0.1

            self.semantic_map.add(obj)

        self._update_meta_gate()
        self._update_meta_slalom()

        for meta_obj in self.semantic_map.get_meta_objects():
            self._clear_around_meta_object(meta_obj)

        self.publish_semantic_map()

        response.added = True
        return response

    def update_candidate_tracks(self, msg: PointCloud2) -> None:
        """
        Update the internal candidate tracks with the latest PointCloud2 message.
        This method is called whenever a new unlabeled candidate track message is received.
        """
        # List of candidate tracks
        tracks: List[np.ndarray] = list(pc2.read_points(msg, field_names=("x", "y"), skip_nans=True))

        # Convert to SlalomCandidate objects with unknown kind
        self.sonar_tracks = SlalomCandidate.build_slalom_candidates_from_sonar_tracks(tracks)

    def add_meta_gate_callback(self,
                               request: AddMetaGate.Request,
                               response: AddMetaGate.Response) -> AddMetaGate.Response:
        """
        Add a meta_object that represents the collection of gate_sides and eventually
        gate_pictures we group together to collectively call a "gate"
        """

        self.semantic_map.add_meta_gate(request.left_gate, request.right_gate)

        self.publish_semantic_map()

        response.added = True
        return response

    def publish_semantic_map(self) -> None:
        """Publish the SemanticMap message"""
        msg = self.semantic_map.create_message()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        self.semantic_map_pub.publish(msg)

    def publish_semantic_map_visual(self) -> None:
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        marker_id = 0

        for obj in self.semantic_map:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = now
            marker.ns = '/pontus'

            marker.id = marker_id
            marker_id += 1

            marker.pose = obj.pose.pose

            self._set_marker_shape(obj, marker)

            marker.action = Marker.ADD

            # Duration it stays around
            marker.lifetime = Duration(seconds=5.5).to_msg()

            marker_array.markers.append(marker)

        self.publish_semantic_map()
        self.semantic_map_visual_pub.publish(marker_array)
        self.publish_semantic_map_debug()

    def publish_semantic_map_debug(self) -> None:
        """Publish a debug string listing how many objects are in each SemanticMap list."""
        sm = self.semantic_map.semantic_map

        debug_msg = String()
        debug_msg.data = (
            "SemanticMap counts | "
            f"gate_image_shark={len(sm.gate_image_shark)}, "
            f"gate_image_fish={len(sm.gate_image_fish)}, "
            f"gate_left={len(sm.gate_left)}, "
            f"gate_right={len(sm.gate_right)}, "
            f"slalom_red={len(sm.slalom_red)}, "
            f"slalom_white={len(sm.slalom_white)}, "
            f"vertical_marker={len(sm.vertical_marker)}, "
            f"bin={len(sm.bin)}, "
            f"octagon={len(sm.octagon)}, "
            f"target={len(sm.target)}, "
            f"meta_gate_set={sm.meta_gate.header.frame_id != ''},"
            f"meta_slalom_row_count={len(sm.meta_slalom.meta_slalom_rows)}"
        )

        self.semantic_map_debug_pub.publish(debug_msg)

    def _update_meta_gate(self) -> None:
        """
        Look at gate side detections and, if a valid pair is found,
        update the SemanticMetaGate in the semantic map.

        Criteria:
        - Use gate_left list
        - Require distance between poles to be ~ gate_width_m within gate_width_tolerance_m.
        - Use body_frame transform to decide which is left vs right.
        """

        # Lock Check: First meta_gate we find we keep set
        if self.semantic_map.semantic_map.meta_gate.header.frame_id != "":
            return

        gate_list = self.semantic_map.semantic_map.gate_left

        if len(gate_list) < 2:
            return

        candidate_pair: list[SemanticObject] | None = None

        # Find a pair whose distance matches the expected gate width
        for i in range(len(gate_list) - 1):
            for j in range(i + 1, len(gate_list)):
                g1 = self._pose_to_vec2(gate_list[i].pose.pose)
                g2 = self._pose_to_vec2(gate_list[j].pose.pose)

                gate_width_est = float(np.linalg.norm(g1 - g2))

                if abs(gate_width_est - self.gate_width_m) <= self.gate_width_tolerance_m:
                    candidate_pair = [gate_list[i], gate_list[j]]

        if candidate_pair is None:
            return

        # Decide which is left/right relative to the robot (body_frame)
        body_frame_poses: list[Pose] = []
        for side in candidate_pair:
            body_frame_side_pose = self._transform_sem_obj_to_body(side)
            body_frame_poses.append(body_frame_side_pose)

        # Higher y in body_frame is "left"
        if body_frame_poses[0].position.y > body_frame_poses[1].position.y:
            left_gate = candidate_pair[0]
            right_gate = candidate_pair[1]
        else:
            left_gate = candidate_pair[1]
            right_gate = candidate_pair[0]

        # for gate_side in candidate_pair:
        #     self._clear_around_meta_object(gate_side)

        # Actually store it in the DC + message
        self.semantic_map.add_meta_gate(left_gate, right_gate)
        self.get_logger().info(
            f"Meta gate updated: left_gate at ({left_gate.pose.pose.position.x:.2f}, "
            f"{left_gate.pose.pose.position.y:.2f}), "
            f"right_gate at ({right_gate.pose.pose.position.x:.2f}, "
            f"{right_gate.pose.pose.position.y:.2f})"
        )

    def _update_meta_slalom(self) -> None:
        """
        Look at detected slalom poles, and use expected widths and tolerances to determine slalom rows
        """

        # TODO: implement behaviour for gate not detected or don't rely on distance to gate for ordering at all
        if self.semantic_map.semantic_map.meta_gate.header.frame_id == "":
            # self.get_logger().info("no gate detected")
            return

        # If all 3 slalom rows are already fully formed (2 white + 1 red), keep them locked.
        existing_rows = self.semantic_map.semantic_map.meta_slalom.meta_slalom_rows
        rows_fully_formed = all(
            len(row.slaloms_white) == 2
            and row.slalom_red.object_type == SemanticObject.SLALOM_RED
            for row in existing_rows
        )
        if len(existing_rows) == 3 and rows_fully_formed:
            return

        red_list: list[SemanticObject] = self.semantic_map.semantic_map.slalom_red
        white_list: list[SemanticObject] = self.semantic_map.semantic_map.slalom_white

        red_candidates = SlalomCandidate.build_slalom_candidates_from_semantic_objects(red_list)
        white_candidates = SlalomCandidate.build_slalom_candidates_from_semantic_objects(white_list)

        all_candidates: list[SlalomCandidate] = []
        all_candidates.extend(red_candidates)
        all_candidates.extend(white_candidates)

        if len(all_candidates) < 1:
            return

        all_candidates.extend(self.sonar_tracks)

        locked_rows = self.semantic_map.semantic_map.meta_slalom.meta_slalom_rows
        ref_pos: Optional[np.ndarray] = None
        ref_line: Optional[np.ndarray] = None
        perp_unit: Optional[np.ndarray] = None
        if locked_rows:
            ref_pos, ref_line = self._row_line_from_semantic(locked_rows[0])
            perp_unit = np.array([-ref_line[1], ref_line[0]])

        proposals: List[SlalomRowProposal] = []

        left_gate = self._pose_to_vec2(self.semantic_map.semantic_map.meta_gate.left_gate.pose.pose)
        flag = False
        for i in range(len(all_candidates)):
            for j in range(i+1, len(all_candidates)):
                p1 = all_candidates[i]
                p2 = all_candidates[j]

                p1_to_p2_vec = p1.pose - p2.pose
                point_width = np.linalg.norm(p1_to_p2_vec)

                point_diff = abs(point_width - self.slalom_white_to_red_width)
                # skip current point pairs if distances are not around 1.5m
                if (not point_diff <= self.slalom_width_tolerance):
                    continue
                
                self.get_logger().info(f"Valid point pair: {point_diff} width diff")

                # Slaloms are in sets of white-red-white so points have to be opposite kind
                if p1.kind == p2.kind: # Remove both unknown pairs. Use kind to still allow for known-known pairs
                    # TODO: Add logic to handle unknown-unknown pairs as potential new rows
                    if p1.kind != CandidateKind.UNKNOWN_TRACK:
                        self.get_logger().warn(f"Invalid point pair: {p1.kind} and {p2.kind} are the same kind")
                        continue

                    if ref_pos is None:
                        continue

                    result = self._classify_unknown_pair(p1, p2, locked_rows[0], ref_line)
                    if result is None:
                        continue
                    white1, red = result
                elif p1.known: # Find one known and assign the other unknown to be the other color
                    white1, red = (p1, p2) if p1.kind == CandidateKind.SLALOM_WHITE else (p2, p1)
                else:
                    white1, red = (p2, p1) if p2.kind == CandidateKind.SLALOM_WHITE else (p1, p2)

                row_line_unit_vec = (red.pose - white1.pose) / np.linalg.norm(red.pose - white1.pose)
                w2_pos = red.pose + (row_line_unit_vec * self.slalom_white_to_red_width)

                # Create a proposal for the slalom row
                proposals.append(SlalomRowProposal(
                    red_pos=red.pose,
                    line_unit_vec=row_line_unit_vec,
                    white1=white1,
                    red=red,
                    white2_pos=w2_pos
                ))

                if flag:
                    continue
            if flag:
                flag = False
                continue
        
        if not proposals:
            return

        proposals = self._dedupe_row_proposals(proposals)
        proposals = self._validate_row_proposals(proposals, ref_pos, ref_line, perp_unit)

        if not proposals:
            return

        slalom_rows = []
        for p in proposals:
            red_to_gate_dist = np.linalg.norm(p.red_pos - left_gate)

            w1_obj = self._promote_candidate(p.white1, SemanticObject.SLALOM_WHITE)
            red_obj = self._promote_candidate(p.red, SemanticObject.SLALOM_RED)

            w2_candidate = SlalomCandidate(kind=CandidateKind.SLALOM_WHITE, pose=p.white2_pos, known=False)
            w2_obj = self._promote_candidate(w2_candidate, SemanticObject.SLALOM_WHITE)

            slalom_rows.append((red_to_gate_dist, [w1_obj, w2_obj], red_obj))  # sort key is arbitrary now, see below

        if (not slalom_rows):
            return

        if (len(slalom_rows) > 3):
            self.get_logger().error("PANIC: Detecting more than 3 slalom rows")
            return

        # order slaloms by distance to the gate
        slalom_rows = sorted(slalom_rows, key=lambda obj: obj[0])

        log_message = "Meta slalom updated:"
        for i in range(len(slalom_rows)):

            log_message += "\n"
            log_message += f"\t[{i}]: \n"
            white_one_pos = f"({slalom_rows[i][1][0].pose.pose.position.x:.2f},{slalom_rows[i][1][0].pose.pose.position.y:.2f})"
            white_two_pos = f"({slalom_rows[i][1][1].pose.pose.position.x:.2f},{slalom_rows[i][1][1].pose.pose.position.y:.2f})"
            red_pos = f"({slalom_rows[i][2].pose.pose.position.x:.2f},{slalom_rows[i][2].pose.pose.position.y:.2f})"
            log_message += f"\t\twhite poles at: {white_one_pos}, {white_two_pos}"
            log_message += "\n"
            log_message += f"\t\tred pole at: {red_pos}"

            row = SemanticMetaSlalomRow()
            row.header = Header()

            row.slaloms_white = slalom_rows[i][1]
            row.slalom_red = slalom_rows[i][2]

            row.header.stamp = row.slalom_red.header.stamp
            row.header.frame_id = row.slalom_red.header.frame_id

            slalom_rows[i] = row

        # log it when we have more rows
        if len(slalom_rows) > len(self.semantic_map.semantic_map.meta_slalom.meta_slalom_rows):
            self.get_logger().info(log_message)
        # add to semantic map
        self.semantic_map.add_meta_slalom(slalom_rows)

    def _classify_unknown_pair(self, p1: SlalomCandidate, p2: SlalomCandidate,
                                locked_row: SemanticMetaSlalomRow,
                                ref_line: np.ndarray) -> Optional[tuple[SlalomCandidate, SlalomCandidate]]:
        """Returns (white1, red) for an unknown-unknown pair, using a locked row as a template. None if ambiguous/invalid."""
        red_locked = self._pose_to_vec2(locked_row.slalom_red.pose.pose)
        whites_locked = [self._pose_to_vec2(w.pose.pose) for w in locked_row.slaloms_white]
        white_projs = sorted(np.dot(w - red_locked, ref_line) for w in whites_locked)
        lo, hi = white_projs[0], white_projs[1]
        tol = self.slalom_white_to_red_width - self.slalom_row_deviation

        proj_p1 = np.dot(p1.pose - red_locked, ref_line)
        proj_p2 = np.dot(p2.pose - red_locked, ref_line)

        p1_inside = (lo + tol) <= proj_p1 <= (hi - tol)
        p2_inside = (lo + tol) <= proj_p2 <= (hi - tol)

        if p1_inside and not p2_inside:
            return p2, p1   # p2 outside -> white, p1 inside -> red
        elif p2_inside and not p1_inside:
            return p1, p2
        else:
            # both inside (near-zero     row deviation — genuinely ambiguous) or both outside (bad pair)
            return None

    def _dedupe_row_proposals(self, proposals: List[SlalomRowProposal]) -> List[SlalomRowProposal]:
        dedup_tolerance = self.slalom_dup_tolerance
        kept: List[SlalomRowProposal] = []

        for p in proposals:
            is_dup = False
            for k in kept:
                if np.linalg.norm(p.red_pos - k.red_pos) < dedup_tolerance:
                    is_dup = True
                    if (p.red.known + p.white1.known) > (k.red.known + k.white1.known):
                        kept.remove(k)
                        kept.append(p)
                    break
            if not is_dup:
                kept.append(p)

        return kept
    
    def _row_line_from_semantic(self, row: SemanticMetaSlalomRow) -> tuple[np.ndarray, np.ndarray]:
        red_pos = self._pose_to_vec2(row.slalom_red.pose.pose)
        white_pos = self._pose_to_vec2(row.slaloms_white[0].pose.pose)
        line_unit = (red_pos - white_pos) / np.linalg.norm(red_pos - white_pos)
        return red_pos, line_unit

    def _validate_row_proposals(self, proposals: List[SlalomRowProposal],
                                ref_pos: Optional[np.ndarray],
                                ref_line: Optional[np.ndarray],
                                perp_unit: Optional[np.ndarray]) -> List[SlalomRowProposal]:
        if ref_pos is None:
            if len(proposals) <= 1:
                # Bootstrap case, nothing to compare against — accept as-is.
                return proposals
            # Bootstrap case with multiple candidates for "first row" — validate against each other.
            ref_pos, ref_line = proposals[0].red_pos, proposals[0].line_unit_vec
            perp_unit = np.array([-ref_line[1], ref_line[0]])

        validated = []
        for p in proposals:
            # parallelism: line directions should align or be exactly opposite (row poles can be walked either way)
            cos_angle = abs(np.dot(p.line_unit_vec, ref_line))
            if cos_angle < math.cos(math.radians(self.slalom_row_deviation_deg)):  # tune as needed
                continue

            offset = p.red_pos - ref_pos
            perp_dist = abs(np.dot(offset, perp_unit))
            along_dist = abs(np.dot(offset, ref_line))

            # perpendicular spacing must be ~0 (same row) or a multiple of row width (adjacent rows)
            spacing_ok = any(
                abs(perp_dist - n * self.slalom_row_width) <= self.slalom_row_tolerance
                for n in (0, 1, 2)
            )
            # rows shouldn't be far offset along their own line direction from one another
            along_ok = along_dist <= self.slalom_row_deviation

            if spacing_ok and along_ok:
                validated.append(p)

        return validated

    def _promote_candidate(self, candidate: SlalomCandidate, object_type: int) -> SemanticObject:
        """
        Return the backing SemanticObject for a known candidate, or build+fuse a new
        one from an unknown candidate's position (geometry-confirmed, camera-unconfirmed).
        """
        if candidate.known:
            return candidate.obj

        now = self.get_clock().now().to_msg()

        obj = SemanticObject()
        obj.header = Header(stamp=now, frame_id="map")
        obj.object_type = object_type

        obj.pose = PoseWithCovariance()
        obj.pose.pose.position = Point(x=float(candidate.pose[0]), y=float(candidate.pose[1]), z=0.0)
        obj.pose.pose.orientation = Quaternion(w=1.0)

        obj.num_detections = 1
        obj.confidence = 0.5   # geometry-only confirmation — lower than a camera-confirmed 1.0
        obj.last_updated = now
        obj.duplicant_tolerance_m = 0.3

        self.semantic_map.add(obj)
        return obj

    def _clear_around_meta_object(self, meta_obj: SemanticObject):

        meta_obj_arr = self._pose_to_vec2(meta_obj.pose.pose)

        for obj in self.semantic_map:

            if obj == meta_obj:
                continue

            obj_pose_arr = self._pose_to_vec2(obj.pose.pose)

            dist_to_meta_obj = np.linalg.norm(meta_obj_arr - obj_pose_arr)

            if dist_to_meta_obj < self.meta_removal_threshold:
                self.semantic_map.remove(obj)

    def _set_marker_shape(self, obj: SemanticObject, marker: Marker) -> None:
        GATE_DIAMETER_VISUAL = 0.15
        SLALOM_DIAMETER_VISUAL = 0.12

        match int(obj.object_type):
            case SemanticObject.GATE_LEFT:
                marker.type = Marker.CYLINDER
                marker.scale.x = GATE_DIAMETER_VISUAL
                marker.scale.y = GATE_DIAMETER_VISUAL
                marker.scale.z = 1.20  # height
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                # marker.type = Marker.MESH_RESOURCE
                # marker.mesh_resource = 'package://pontus_mapping/visual_meshes/LeftGate.obj'
                # marker.scale.x = 1.0
                # marker.scale.y = 1.0
                # marker.scale.z = 1.0
                # marker.mesh_use_embedded_materials = True

            case SemanticObject.GATE_RIGHT:
                marker.type = Marker.CYLINDER
                marker.scale.x = GATE_DIAMETER_VISUAL
                marker.scale.y = GATE_DIAMETER_VISUAL
                marker.scale.z = 1.20  # height
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                # marker.type = Marker.MESH_RESOURCE
                # marker.mesh_resource = 'package://pontus_mapping/visual_meshes/RightGate.obj'
                # marker.scale.x = 1.0
                # marker.scale.y = 1.0
                # marker.scale.z = 1.0
                # marker.mesh_use_embedded_materials = True

            case SemanticObject.VERTICAL_MARKER:
                # marker.type = Marker.MESH_RESOURCE
                # marker.mesh_resource = 'package://pontus_mapping/visual_meshes/VerticalMarker.obj'
                # marker.scale.x = 1.0
                # marker.scale.y = 1.0
                # marker.scale.z = 1.0
                # marker.mesh_use_embedded_materials = True
                marker.type = Marker.CYLINDER
                marker.scale.x = 0.25
                marker.scale.y = 0.25
                marker.scale.z = 3.0  # height
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

            case SemanticObject.GATE_IMAGE_SHARK:
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = 'package://pontus_mapping/visual_meshes/shark_marker.obj'
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0

                marker.mesh_use_embedded_materials = True

            case SemanticObject.GATE_IMAGE_FISH:
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = 'package://pontus_mapping/visual_meshes/fish_marker.obj'
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0

                marker.mesh_use_embedded_materials = True

            case SemanticObject.SLALOM_RED:
                marker.type = Marker.CYLINDER
                marker.scale.x = SLALOM_DIAMETER_VISUAL
                marker.scale.y = SLALOM_DIAMETER_VISUAL
                marker.scale.z = 0.90  # height
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

            case SemanticObject.SLALOM_WHITE:
                marker.type = Marker.CYLINDER
                marker.scale.x = SLALOM_DIAMETER_VISUAL
                marker.scale.y = SLALOM_DIAMETER_VISUAL
                marker.scale.z = 0.90
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0

            case SemanticObject.OCTAGON:
                # (use a cube for now; meshes can be flaky in Foxglove)
                marker.type = Marker.CUBE
                marker.scale.x = 0.70
                marker.scale.y = 0.70
                marker.scale.z = 0.70
                marker.color.r = 0.9
                marker.color.g = 0.7
                marker.color.b = 0.1
                marker.color.a = 1.0

            case SemanticObject.BIN:
                marker.type = Marker.CUBE
                marker.scale.x = 0.40
                marker.scale.y = 0.40
                marker.scale.z = 0.40
                marker.color.r = 0.2
                marker.color.g = 0.6
                marker.color.b = 0.9
                marker.color.a = 1.0

            case SemanticObject.TARGET:
                marker.type = Marker.CUBE
                marker.scale.x = 0.60
                marker.scale.y = 0.60
                marker.scale.z = 1.10
                marker.color.r = 0.8
                marker.color.g = 0.2
                marker.color.b = 0.1
                marker.color.a = 1.0

            case _:
                self.get_logger().info('Found marker with unknown object type, skipping')
                marker.action = Marker.DELETE

    def _pose_to_vec2(self, pose: Pose) -> np.ndarray:
        """Convert a Pose into 2D np.array [x, y] for planar distance calculations."""
        return np.array([pose.position.x, pose.position.y], dtype=float)

    def _transform_sem_obj_to_body(self, obj: SemanticObject) -> Pose:
        """
        Transform a SemanticObject's pose to the robot's body_frame.

        This mimics the logic in PrequalGateTask so meta_gate.left/right
        are defined in the same way (left/right in the body frame).
        """
        pose_stamped = PoseStamped()
        pose_stamped.header = obj.header
        pose_stamped.pose = obj.pose.pose

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='body_link',
                source_frame=pose_stamped.header.frame_id,
                time=Time(
                    seconds=pose_stamped.header.stamp.sec,
                    nanoseconds=pose_stamped.header.stamp.nanosec,
                )
            )

            pose_transformed_stamped = tf2_geometry_msgs.do_transform_pose(
                pose_stamped,
                transform
            )

            return pose_transformed_stamped.pose

        except Exception as e:
            self.get_logger().warn(
                f"Failed to transform semantic object to body_link"
                f"(current frame: {obj.header.frame_id})"
            )
            self.get_logger().warn(f"exception: {e}")
            return pose_stamped.pose


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
