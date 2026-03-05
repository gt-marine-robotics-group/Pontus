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


class OctagonSurfaceTask(BaseTask):

    def __init__(self):
        super().__init__("octagon_surface_task")

        self.service_callback_group = MutuallyExclusiveCallbackGroup()

        # ------ Parameters ------
        self.declare_parameters(
            namespace='',
            parameters=[
                ('height_from_bottom', 1.0),
                ('pool_depth', 2.0),
                ('follow_path_period', 0.1),
                ('table_height', 1.0),
                ('waypoint_height_above_table', 0.75),
                ('lead_in_dist', 1.0),
                ('num_table_detections_threshold', 1)

            ]
        )

        self.height_from_bottom_m: float = float(
            self.get_parameter('height_from_bottom').value)

        self.pool_depth: float = float(self.get_parameter('pool_depth').value)

        self.follow_path_period: float = float(
            self.get_parameter('follow_path_period').value)

        self.table_height = float(self.get_parameter('table_height').value)

        self.waypoint_height_above_table = float(
            self.get_parameter('waypoint_height_above_table').value)

        self.lead_in_dist = float(self.get_parameter('lead_in_dist').value)

        self.num_table_detections_threshold = int(self.get_parameter(
            'num_table_detections_threshold').value)

        # ------ Variables ------
        self.waypoints_are_created: bool = False
        self.path: list[np.ndarray] = []

        self.curr_waypoint: Pose = None

        self.execute_path: bool = False

        self.latest_odom = None
        self.latest_semantic_map = None

        self.table_detection = None

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

        if self.execute_path:
            return

        if len(msg.octagon) == 0:
            return

        max_detections = 0
        best_table = None

        for table in msg.octagon:
            if table.num_detections > max_detections:
                max_detections = table.num_detections
                best_table = table

        self.get_logger().info(
            f"Octagon table found with {best_table.num_detections} detections")

        if (best_table is not None
            and best_table.num_detections >= self.num_table_detections_threshold
                and self.latest_odom is not None):

            self.table_detection = best_table
            path = self.generate_waypoints(self.table_detection)

            if path is not None:
                self.waypoints_are_created = True
                self.path = path
                self.execute_path = True

        self.latest_semantic_map = msg

    def generate_waypoints(self, table_obj: SemanticObject) -> list[np.ndarray]:
        """
        We generate a lead in point that is at the right point going into the
        table and a set 'lead_in_point_dist' away.

        We then make a point right over the top of the detected table object.
            - Later we may need to add an offset to account for the fact
              that the table detection is biased to the front of the object
              and not truly centered where it should be

        We then trigger software E-Stop for a set duration
            - This may sound dumb but is actually more reliable than sending
              a hard coded waypoint to go to
            - It can be easy to send a point that is just above the pool in
              which case it would be impossible for the robot to get to and
              would get stuck. Triggering the software E-Stop ensures we surface
              and then easily recover when the E-Stop turns back on

        Args:
            table_obj [SemanticObject] : octagon table object with most detections
                                         in semantic map

        Return:
            list[np.ndarray] : Path for the robot to follow as 2D np.array 
                               vectors
        """

        robot_xy = self._pose_to_nparray(self.latest_odom.pose.pose)
        table_pos_xy = self._pose_to_nparray(table_obj.pose.pose)

        table_to_robot_vec = robot_xy - table_pos_xy
        table_to_sub_dir = table_to_robot_vec / \
            np.linalg.norm(table_to_robot_vec)

        z_depth_above_table = -self.pool_depth + \
            self.table_height + self.waypoint_height_above_table

        lead_in_wp = np.zeros((3,))
        lead_in_wp[0:2] = table_pos_xy + table_to_sub_dir * self.lead_in_dist
        lead_in_wp[2] = z_depth_above_table

        above_table_wp = np.zeros((3,))
        above_table_wp[0:2] = table_pos_xy
        above_table_wp[2] = z_depth_above_table

        # TODO: Turn on software E-Stop

        # Surface with hard waypoint
        surface_wp = np.zeros((3,))
        surface_wp[0:2] = table_pos_xy
        surface_wp[2] = 1.0

        # Approximately going back towards the gate before ending the task
        # to ensure we are outside the area we would collide with the table

        if self.latest_semantic_map is not None and self.latest_semantic_map.meta_gate is not None:
            gate_pos_xy = self._pose_to_nparray(
                self.latest_semantic_map.meta_gate.left_gate.pose.pose)
            table_to_gate_vec = table_pos_xy - gate_pos_xy
            table_to_gate_dir = table_to_gate_vec / \
                np.linalg.norm(table_to_gate_vec)

            lead_out_wp = np.zeros((3,))
            lead_out_wp[0:2] = table_pos_xy + \
                table_to_gate_dir * self.lead_in_dist
            lead_out_wp[2] = z_depth_above_table
        else:
            lead_out_wp = lead_in_wp

        return [lead_in_wp, above_table_wp, surface_wp, above_table_wp, lead_out_wp]

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

            self.get_logger().info(f"Going to: {target_pos_xy}")
            self._send_waypoint_command(target_pos_xy)

    def _send_waypoint_command(self, target_pos_xyz: np.ndarray) -> None:
        """
        Convert a np.ndarray 2D vector to a command pose and send to pos_controller
        """
        cmd_pose = Pose()

        cmd_pose.position.x = target_pos_xyz[0]
        cmd_pose.position.y = target_pos_xyz[1]
        cmd_pose.position.z = target_pos_xyz[2]

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
