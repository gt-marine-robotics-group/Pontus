import numpy as np
from enum import Enum
from dataclasses import dataclass

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj

from pontus_msgs.msg import SemanticMap, SemanticObject


class GateSide(Enum):
    RIGHT = 0
    LEFT = 1


@dataclass
class GatePair:
    """
    Holds information about gate objects selected to be a gate pair
    """
    left_gate: SemanticObject
    right_gate: SemanticObject


class PrequalGateTask(BaseTask):

    def __init__(self):
        super().__init__("prequal_gate_task")

        self.service_callback_group = MutuallyExclusiveCallbackGroup()

        # ------ Parameters ------
        self.declare_parameters(
            namespace='',
            parameters=[
                ('height_from_bottom', 0.5),
                ('gate_side', 0),  # Gate side right
                # How far should the apparoach and pass through points be to the gate
                ('waypoint_dist_from_gate', 0.6),
                ('follow_path_period', 0.25),
                ('pool_depth', 2.0)
            ]
        )

        self.height_from_bottom_m: float = float(
            self.get_parameter('height_from_bottom').value)
        self.gate_side: GateSide = GateSide(
            self.get_parameter('gate_side').value)
        self.waypoint_dist_from_gate_m: float = float(
            self.get_parameter('waypoint_dist_from_gate').value)
        self.follow_path_period: float = float(
            self.get_parameter('follow_path_period').value)
        self.pool_depth: float = float(self.get_parameter('pool_depth').value)

        # ------ Variables ------
        self.waypoints_are_created: bool = False
        self.path: list[np.ndarray] = []

        self.curr_waypoint: Pose = None

        self.execute_path: bool = False

        self.detected_gate_pair: GatePair = None

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
        the Gate has been detected.

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

        if msg.meta_gate.header.frame_id == "":
            return

        self.detected_gate_pair = GatePair(
            left_gate=msg.meta_gate.left_gate,
            right_gate=msg.meta_gate.right_gate
        )

        if self.detected_gate_pair is not None and self.latest_odom is not None:
            path = self.generate_waypoints(self.detected_gate_pair)

            if path is not None:
                self.waypoints_are_created = True
                self.path = path
                self.execute_path = True

    def generate_waypoints(self, gate_pair: GatePair) -> list[np.ndarray]:
        """
        Once we have a GatePair we can generate the waypoints for the sub
        to pass through with some basic linalg.

        We set the point where we want the sub to pass through the gate.
        This is the midpoint of the gate midpoint and one of the gate sides.
            - Which gate side is decided before the run

        We then find the perpendicular unit vector to generate two waypoints
        and return these as the path for the robot to follow.

        Args:
            gate_pair [GatePair] : detected pair from the semantic map

        Return:
            list[np.ndarray] : Path for the robot to follow as 2D np.array 
                               vectors
        """

        g1: np.ndarray = self._pose_to_nparray(gate_pair.left_gate.pose.pose)
        g2: np.ndarray = self._pose_to_nparray(gate_pair.right_gate.pose.pose)

        # gate passthrough is the point the sub actually goes through the gate
        if self.gate_side == GateSide.LEFT:
            # This is the midpoint between the midpoint and on of the gate sides
            passthrough_point = (3*g1 + g2) / 4.0
        else:
            passthrough_point = (3*g2 + g1) / 4.0

        # Gate vector from left side to right side
        gate_vec = g2 - g1

        # Perpendicular Vector
        perp_vec = np.array([-gate_vec[1], gate_vec[0]])
        perp_unit_vec = perp_vec / np.linalg.norm(perp_vec)

        waypoint_delta = self.waypoint_dist_from_gate_m * perp_unit_vec

        waypoint_1 = passthrough_point + waypoint_delta
        waypoint_2 = passthrough_point - waypoint_delta

        robot_xy = self._pose_to_nparray(self.latest_odom.pose.pose)

        wp1_dist = np.linalg.norm(waypoint_1 - robot_xy)
        wp2_dist = np.linalg.norm(waypoint_2 - robot_xy)

        if wp1_dist < wp2_dist:
            return [waypoint_1, waypoint_2]
        else:
            return [waypoint_2, waypoint_1]

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

        self.go_to_pose_client.go_to_pose(PoseObj(cmd_pose=cmd_pose,
                                                  skip_orientation=True))

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
