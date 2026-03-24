import rclpy
import math
import numpy as np
import tf_transformations
from enum import Enum
from typing import Optional

from geometry_msgs.msg import Pose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj

from pontus_msgs.msg import SemanticMap
from pontus_msgs.srv import AddSemanticObject

class SearchConditions(Enum):
    GATE = 0
    SLALOM = 1
    TARGET = 2
    BIN = 3
    OCTAGON = 4
    MARKER = 5


class ScanTask(BaseTask):

    def __init__(
        self, 
        target_angle1_rad: float = math.radians(45),
        target_angle2_rad: float = math.radians(-45),
        terminating_condition: SearchConditions = SearchConditions.GATE,
        fallback_points=None):
        super().__init__("prequal_search_gate_task")

        self.service_callback_group = MutuallyExclusiveCallbackGroup()

        self.terminating_condition = terminating_condition

        self.fallback_points = fallback_points

        # ----- Search Configuration -----
        self.search_angles = [target_angle1_rad, target_angle2_rad]
        self.search_index = 0
        self.start_turn = False

        self.turn_interval = 4.0  # seconds
        self.last_turn_time = self.get_clock().now()

        # ROS Subscriptions
        self.semantic_map_sub = self.create_subscription(
            SemanticMap,
            '/pontus/semantic_map',
            self.semantic_map_callback,
            10
        )

        # Action/Service Clients
        self.go_to_pose_client = GoToPoseClient(self)

        self.add_semantic_object_client = self.create_client(
            AddSemanticObject,
            '/pontus/add_semantic_object',
        )

        self.get_logger().info("Finished Setting Up")

        # Timer for turning behavior
        self.turn_timer = self.create_timer(
            0.5,
            self.turn_callback,
            self.service_callback_group
        )

    def semantic_map_callback(self, msg: SemanticMap) -> None:
        """
        If a gate is detected, stop the search immediately.
        """

        if self.terminating_condition is SearchConditions.GATE: 
            if msg.meta_gate.header.frame_id != "":
                self.get_logger().info("Gate pair detected in semantic map")
                self.complete(True)

        elif self.terminating_condition is SearchConditions.SLALOM:
            if msg.meta_slalom.header.frame_id != "":
                self.get_logger().info("Slalom Pair detected in semantic map")
                self.complete(True)

        elif self.terminating_condition is SearchConditions.MARKER:
            if msg.meta_gate.header.frame_id == "":
                return

            marker = self.detect_marker(msg)
            if marker is not None:
                self.get_logger().info("Vertical marker detected in semantic map")
                self.complete(True)

    def turn_callback(self) -> None:
        """
        Follows the given target angles 
        """

        # Wait for previous motion to finish
        if not self.start_turn or self.go_to_pose_client.at_pose():

            # Enforce dwell time between turns
            now = self.get_clock().now()
            dt = (now - self.last_turn_time).nanoseconds * 1e-9
            if dt < self.turn_interval:
                return

            self.last_turn_time = now
            self.start_turn = True

            target_angle = self.search_angles[self.search_index]

            self.get_logger().info(
                f"Turning {'right' if target_angle > 0 else 'left'} "
                f"{math.degrees(abs(target_angle))} degrees"
            )

            # Send turn command (RELATIVE)
            self.turn_command(target_angle)

            # Alternate index
            self.search_index = (self.search_index + 1) % len(self.search_angles)

    def _send_fallback_semantic_objects(self) -> None:
        """
        Sends fallback semantic objects if needed.
        """
        if not self.add_semantic_object_client.service_is_ready():
            self.get_logger().warn("AddSemanticObject service not available")
            return

        req = AddSemanticObject.Request()
        req.ids = [self.fallback_points[0][0], self.fallback_points[1][0]]
        req.positions = [self.fallback_points[0][1], self.fallback_points[1][1]]

        self.add_semantic_object_client.call_async(req)

    def detect_marker(self, sem_map: SemanticMap) -> Optional[np.ndarray]:
        """
        Find a vertical-marker-like candidate using gate geometry.
        """
        _, gate_unit_norm, gate_midpoint = self._get_gate_unit_normal(sem_map)

        best_candidate_marker: Optional[np.ndarray] = None
        best_dist: Optional[float] = None

        for marker in sem_map.gate_left:
            marker_vec = self._pose_to_nparray(marker.pose.pose)
            marker_gate_vec = marker_vec - gate_midpoint

            parallel = np.dot(marker_gate_vec, gate_unit_norm) * gate_unit_norm
            perp = marker_gate_vec - parallel
            dist = float(np.linalg.norm(perp))

            dist_from_gate = np.linalg.norm(marker_gate_vec)
            if dist <= 1.5 and dist_from_gate >= 1.8:
                if best_dist is None or dist < best_dist:
                    best_dist = dist
                    best_candidate_marker = marker_vec

        return best_candidate_marker

    def _get_gate_unit_normal(self, sem_map: SemanticMap) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        g1 = self._pose_to_nparray(sem_map.meta_gate.left_gate.pose.pose)
        g2 = self._pose_to_nparray(sem_map.meta_gate.right_gate.pose.pose)

        gate_vec = g2 - g1
        gate_unit_vec = gate_vec / np.linalg.norm(gate_vec)

        perp_vec = np.array([-gate_vec[1], gate_vec[0]])
        perp_unit_vec = perp_vec / np.linalg.norm(perp_vec)

        midpoint = (g1 + g2) / 2.0

        return gate_unit_vec, perp_unit_vec, midpoint

    def _pose_to_nparray(self, pose: Pose) -> np.ndarray:
        return np.array([pose.position.x, pose.position.y], dtype=float)

    def turn_command(self, relative_yaw: float) -> None:
        """
        Sends a RELATIVE rotation command.
        """

        cmd_pose = Pose()

        # Convert yaw to quaternion
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(
            0.0, 0.0, relative_yaw
        )

        # No positional movement
        cmd_pose.position.x = 0.0
        cmd_pose.position.y = 0.0
        cmd_pose.position.z = 0.0

        cmd_pose.orientation.x = qx
        cmd_pose.orientation.y = qy
        cmd_pose.orientation.z = qz
        cmd_pose.orientation.w = qw

        self.go_to_pose_client.go_to_pose(
            pose_obj=PoseObj(
                cmd_pose=cmd_pose,
                use_relative_position=True,
                skip_orientation=False
            )
        )
