import numpy as np
from enum import Enum 
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj

from pontus_msgs.msg import SemanticMap, SemanticObject, SemanticMetaGate

class MarkerSide(Enum):
    RIGHT = 0
    LEFT  = 1

class PrequalVerticalMarkerTask(BaseTask):
    
    def __init__(self):
        super().__init__("prequal_vertical_marker_task")

        self.service_callback_group = MutuallyExclusiveCallbackGroup()

        self.declare_parameters(
            namespace='',
            parameters=[
                ('height_from_bottom', 0.5),
                ('pool_depth', 2.0),
                ('waypoint_dist_from_marker', 0.8),
                ('waypoint_dist_from_gate', 1.0),
                ('marker_centerline_tolerance', 0.8),
                ('follow_path_period', 0.25)
            ]
        )

        self.height_from_bottom_m          : float = float(self.get_parameter('height_from_bottom').value)
        self.pool_depth                    : float = float(self.get_parameter('pool_depth').value)
        self.waypoint_dist_from_marker_m   : float = float(self.get_parameter('waypoint_dist_from_marker').value)
        self.waypoint_dist_from_gate_m     : float = float(self.get_parameter('waypoint_dist_from_gate').value)
        self.marker_centerline_tolerance_m : float = float(self.get_parameter('marker_centerline_tolerance').value)
        self.follow_path_period            : float = float(self.get_parameter('follow_path_period').value)
        
        # ------ State Variables ------
        self.waypoints_are_created : bool = False
        self.path : list[np.ndarray] = []

        self.curr_waypoint: Pose = None 

        self.execute_path : bool = False
        
        self.gate_pair : Optional[SemanticMetaGate] = None
        self.detected_marker : Optional[np.ndarray] = None

        self.latest_odom = None

        # ------ ROS Subscriptions ------
        self.semantic_map_sub = self.create_subscription(
            SemanticMap,
            '/pontus/semantic_map',
            self.semantic_map_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odom_callback,
            10
        )

        # ------ ROS Action / Service Managers ------
        self.go_to_pose_client = GoToPoseClient(self)

        # ------ TF Transforms ------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ------ Timers ------
        self.follow_path_timer = self.create_timer(
            self.follow_path_period,
            self.follow_path,
            self.service_callback_group
        )

    def odom_callback(self, msg: Odometry) -> None:
        """
        Keep track of current position of the robot
        """

        self.latest_odom = msg

    def semantic_map_callback(self, msg: SemanticMap) -> None:
        """
        When the semantic map is updated check for the conditions to show
        the Marker has been detected.
        
        If it has been detected calculate the waypoints to travel through
        the gate.
        
        Args:
            msg [SemanticMap] : contains semantic information about position
                                and labels of relavant obstacles in the course 
        
        Return:
            N/A
        """

        if self.waypoints_are_created:
            return

        if self.gate_pair is None:
            self.gate_pair = msg.meta_gate

        self.detected_marker = self.detect_marker(msg)

        if self.detected_marker is not None:
            path = self.generate_waypoints(self.detected_marker)

            if path is not None:
                self.waypoints_are_created = True
                self.path = path
                self.execute_path = True

    def detect_marker(self, sem_map : SemanticMap) -> Optional[np.ndarray]:
        """
        Given the semantic map. Check all gate side detections to see if
        any pair meet the conditions to be a vertical marker
        
        - How close does it lie to being in line with going through the 
          centerline of the gate. 

        Args:
            sem_map [SemanticMap] : Map containing all detected semantic objects
        
        Return:
            np.ndarray of marker [x, y] in map frame if found,
            otherwise None.
        """
        
        _, gate_unit_norm, gate_midpoint = self._get_gate_unit_normal()

        best_candidate_marker: Optional[np.ndarray] = None
        best_dist: Optional[float] = None

        for marker in sem_map.vertical_marker:
            marker_vec: np.ndarray = self._pose_to_nparray(marker.pose.pose)

            marker_gate_vec = marker_vec - gate_midpoint
            
            parallel = np.dot(marker_gate_vec, gate_unit_norm) * gate_unit_norm
            perp = marker_gate_vec - parallel
            dist = float(np.linalg.norm(perp))

            if dist <= self.marker_centerline_tolerance_m:
                if best_dist is None or dist < best_dist:
                    best_dist = dist
                    best_candidate_marker = marker_vec

        return best_candidate_marker

    def generate_waypoints(self, marker_xy: np.ndarray) -> list[np.ndarray]:

        gate_unit_vec, gate_unit_norm, gate_midpoint = self._get_gate_unit_normal()
        
        # Around the marker
        waypoint_vm_1 = marker_xy + gate_unit_vec * self.waypoint_dist_from_marker_m
        waypoint_vm_2 = marker_xy + gate_unit_norm * self.waypoint_dist_from_marker_m
        waypoint_vm_3 = marker_xy - gate_unit_vec * self.waypoint_dist_from_marker_m

        waypoint_return_1 = gate_midpoint + gate_unit_norm * self.waypoint_dist_from_gate_m
        waypoint_return_2 = gate_midpoint - gate_unit_norm * self.waypoint_dist_from_gate_m

        robot_xy = self._pose_to_nparray(self.latest_odom.pose.pose)

        wp1_dist = np.linalg.norm(waypoint_vm_1 - robot_xy)
        wp3_dist = np.linalg.norm(waypoint_vm_3 - robot_xy)

        if wp1_dist < wp3_dist:
            return [waypoint_vm_1, waypoint_vm_2, waypoint_vm_3, waypoint_return_1, waypoint_return_2]
        else:
            return [waypoint_vm_3, waypoint_vm_2, waypoint_vm_1, waypoint_return_1, waypoint_return_2]
        
    def follow_path(self) -> None:
        """
        After we generate the waypoints we switch to execute mode and follow
        the path we have created.
        
        Once we reach the end of this path we exit the task.
        """

        if not self.execute_path:
            return

        if self.curr_waypoint is None or self.go_to_pose_client.at_pose():
            if not self.path:
                self.complete(True)
                return

            target_pos_xy = self.path.pop(0)
            self._send_waypoint_command(target_pos_xy)

    
    def _send_waypoint_command(self, target_pos_xy: np.ndarray) -> None:
        """
        Convert a np.ndarray 2D vector to a command pose and send to pos_controller
        """
        cmd_pose = Pose()
        
        cmd_pose.position.x = target_pos_xy[0]
        cmd_pose.position.y = target_pos_xy[1]
        cmd_pose.position.z = -self.pool_depth + self.height_from_bottom_m

        self.curr_waypoint = cmd_pose

        self.go_to_pose_client.go_to_pose(PoseObj(cmd_pose=cmd_pose))

    def _get_gate_unit_normal(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        
        g1 : np.ndarray = self._pose_to_nparray(self.gate_pair.left_gate.pose.pose)
        g2 : np.ndarray = self._pose_to_nparray(self.gate_pair.right_gate.pose.pose)

        # Gate vector from left side to right side
        gate_vec = g2 - g1
        gate_unit_vec = gate_vec / np.linalg.norm(gate_vec)

        # Perpendicular Vector
        perp_vec = np.array([-gate_vec[1], gate_vec[0]])
        perp_unit_vec = perp_vec / np.linalg.norm(perp_vec)

        midpoint = (g1 + g2) / 2.0

        return gate_unit_vec, perp_unit_vec, midpoint

    def _pose_to_nparray(self, msg: Pose) -> np.ndarray:
        """
        Convert a Pose into a 2D numpy array
        
        Args:
            msg [Pose]
        Return:
            [np.ndarray] : Vector version of the ROS Pose for easy math
        """
        return np.array([
            msg.position.x,
            msg.position.y],
            dtype=float
        )
        
