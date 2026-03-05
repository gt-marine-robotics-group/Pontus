from enum import Enum
import math
import numpy as np
from typing import Optional, List
from dataclasses import dataclass
import tf2_geometry_msgs
import tf2_ros

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from rclpy.time import Time


from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj
from pontus_msgs.msg import SemanticObject, SemanticMap, SemanticMetaGate, SemanticMetaSlalomRow, SemanticMetaSlalom
import tf_transformations


class SlalomSide(Enum):
    RIGHT = 0
    LEFT = 1


@dataclass
class SlalomRow:
    pole_left: SemanticObject
    pole_middle: SemanticObject
    pole_right: SemanticObject


class SlalomTask(BaseTask):
    def __init__(self):
        super().__init__("slalom_task")

        # TODO: Functionality to perform slalom from reverse side.

        # ------ Parameters ------
        self.declare_parameters(
            namespace='',
            parameters=[
                ('height_from_bottom', 1.0),
                ('slalom_side', 0),  # Go on the right of the red pole
                # How far should the apparoach and pass through points be to the slalom poles
                ('waypoint_dist_from_pole', 0.4),
                ('require_all_rows', False),
                ('start_gate_side', True),
                ('follow_path_period', 0.25),
                ('pool_depth', 2.0)
            ]
        )

        # Need this to prevent deadlock issues
        self.service_callback_group = MutuallyExclusiveCallbackGroup()

        self.height_from_bottom_m: float = float(
            self.get_parameter('height_from_bottom').value)
        self.slalom_side: SlalomSide = SlalomSide(
            self.get_parameter('slalom_side').value)
        self.waypoint_dist_from_pole: float = float(
            self.get_parameter('waypoint_dist_from_pole').value)
        self.follow_path_period: float = float(
            self.get_parameter('follow_path_period').value)
        self.pool_depth: float = float(self.get_parameter('pool_depth').value)

        self.require_all_rows = self.get_parameter('require_all_rows').value

        self.start_gate_side = self.get_parameter('start_gate_side').value

        # Abstracted go to pose client
        self.go_to_pose_client = GoToPoseClient(self)

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

        self.slalom_debug_pub = self.create_publisher(
            String,
            '/pontus/slalom_debug',
            10
        )

        self.follow_path_timer = self.create_timer(
            self.follow_path_period,
            self.follow_path,
            self.service_callback_group
        )

        self.path = []

        self.curr_waypoint = None

        self.execute_path = False

        self.execute_turn = False

        self.turned_once = False

        self.turning = False

        self.latest_odom = None

        self.detected_slalom_rows = []

        self.rows_passed = 0

        self.num_waypoints_passed = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("Starting Slalom Task")

    # Callbacks

    def odom_callback(self, msg: Odometry) -> None:
        """
        Handle odom callback.

        Keeps track of current odometry. On the first odometry message, will set the desired depth
        as the current depth.

        Args:
        ----
        msg (Odometry): odometry message from /pontus/odometry

        Return:
        ------
        None

        """
        self.latest_odom = msg


    def semantic_map_callback(self, msg: SemanticMap) -> None:
        # If we're already executing a plan, don't rebuild/regenerate.
        # (But allow regeneration once the path is empty again.)
        if len(self.detected_slalom_rows) == 3 and self.path:
            return

        if self.rows_passed >= 3:
            return

        if self.require_all_rows and len(msg.meta_slalom.meta_slalom_rows) < 3:
            self.get_logger().warn(
                f"requiring all slalom rows to be detected to proceed. Only {len(msg.meta_slalom.meta_slalom_rows)} detected."
            )
            return

        meta_slalom_rows: List[SemanticMetaSlalomRow] = list(msg.meta_slalom.meta_slalom_rows)

        self.detected_slalom_rows = []

        if not self.start_gate_side:
            meta_slalom_rows.reverse()

        for meta_row in meta_slalom_rows:
            white_1_body = self._transform_sem_obj_to_body(meta_row.slaloms_white[0])
            white_2_body = self._transform_sem_obj_to_body(meta_row.slaloms_white[1])

            if white_1_body.position.y > white_2_body.position.y:
                pole_left = meta_row.slaloms_white[0]
                pole_right = meta_row.slaloms_white[1]
            else:
                pole_left = meta_row.slaloms_white[1]
                pole_right = meta_row.slaloms_white[0]

            self.detected_slalom_rows.append(
                SlalomRow(
                    pole_left=pole_left,
                    pole_middle=meta_row.slalom_red,
                    pole_right=pole_right
                )
            )

        # Generate new waypoints only when we don't currently have a path to execute
        if self.latest_odom is not None and not self.path and not self.execute_path:
            path = self.generate_waypoints(self.detected_slalom_rows[self.rows_passed:])

            if path:
                self.path = path
                self.execute_path = True
                self.execute_turn = False
            else:
                self.execute_turn = True

    def publish_slalom_debug(self) -> None:
        debug_msg = String()

        debug_msg.data = (
            "Slalom stuff | "
            f"waypoints passed: {self.num_waypoints_passed},"
            f"rows_passed: {self.rows_passed},"
            f"current path length: {len(self.path)},"
            f"detected rows: {len(self.detected_slalom_rows)},"
            f"path: {str(self.path)},"
            f"turning: {self.turning},"
            f"turned_once: {self.turned_once},"
            f"execute_turn: {self.execute_turn}"
        )

        self.slalom_debug_pub.publish(debug_msg)


    def follow_path(self) -> None:
        self.publish_slalom_debug()

        # Handle turning behavior
        if self.execute_turn and not self.turned_once:
            if not self.turning:
                self.turn(self.slalom_side)
                self.turning = True
            elif self.execute_path:
                self.turned_once = True
                self.turning = False
                self.execute_turn = False
            return

        if not self.execute_path:
            return

        if not self.path:
            # No more waypoints in current plan
            self.execute_path = False
            return

        # First waypoint: command it (do NOT pop yet)
        if self.curr_waypoint is None:
            self._send_waypoint_command(self.path[0])
            return

        # If we reached the current waypoint, advance to next
        if self.go_to_pose_client.at_pose():
            self.path.pop(0)
            self.num_waypoints_passed += 1
            self.get_logger().info(f"num_waypoints passed: {self.num_waypoints_passed}")

            if (self.num_waypoints_passed % 2 == 0):
                self.rows_passed += 1
                self.get_logger().info(f"incremented num_rows: {self.rows_passed}")
                self.turned_once = False  # reset per-row behavior

            if not self.path:
                if self.rows_passed >= 3:
                    self.complete(True)
                else:
                    self.execute_path = False
                    self.curr_waypoint = None
                return

            # Command next waypoint
            self._send_waypoint_command(self.path[0])
    

    def generate_waypoints(self, slalom_rows: List[SlalomRow]) -> list[np.ndarray]:

        path = []

        if not slalom_rows:
            return path

        robot_xy = self._pose_to_nparray(self.latest_odom.pose.pose)

        for idx, slalom_row in enumerate(slalom_rows):
            if self.slalom_side == SlalomSide.LEFT:
                p1 = self._pose_to_nparray(slalom_row.pole_left.pose.pose)
                p2 = self._pose_to_nparray(slalom_row.pole_middle.pose.pose)
            else:
                p1 = self._pose_to_nparray(slalom_row.pole_middle.pose.pose)
                p2 = self._pose_to_nparray(slalom_row.pole_right.pose.pose)

            row_vec = p2 - p1

            perp_vec = np.array([-row_vec[1], row_vec[0]])
            perp_unit_vec = perp_vec / np.linalg.norm(perp_vec)

            passthrough_point = (p1 + p2) / 2

            waypoint_dist = self.waypoint_dist_from_pole

            waypoint_delta = waypoint_dist * perp_unit_vec
            waypoint_1 = passthrough_point + waypoint_delta
            waypoint_2 = passthrough_point - waypoint_delta

            wp1_dist = np.linalg.norm(waypoint_1 - robot_xy)
            wp2_dist = np.linalg.norm(waypoint_2 - robot_xy)

            if wp1_dist < wp2_dist:
                path.extend([waypoint_1, waypoint_2])
            else:
                path.extend([waypoint_2, waypoint_1])

        return path

    def turn(self, slalom_side: SlalomSide):

        cmd_pose = Pose()
        if slalom_side == SlalomSide.LEFT:
            # turn right
            yaw_abs = -0.25 * math.pi
        else:
            # turn left
            yaw_abs = 0.25 * math.pi

        quat = tf_transformations.quaternion_from_euler(
            0.0, 0.0, yaw_abs)

        cmd_pose.position.x = self.latest_odom.pose.pose.position.x
        cmd_pose.position.y = self.latest_odom.pose.pose.position.y
        cmd_pose.position.z = self.latest_odom.pose.pose.position.z

        cmd_pose.orientation.x = quat[0]
        cmd_pose.orientation.y = quat[1]
        cmd_pose.orientation.z = quat[2]
        cmd_pose.orientation.w = quat[3]

        self.go_to_pose_client.go_to_pose(
            PoseObj(cmd_pose=cmd_pose, skip_orientation=False))

    def _send_waypoint_command(self, target_pos_xy: np.ndarray) -> None:
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

    def _transform_sem_obj_to_body(self, obj: SemanticObject) -> Pose:
        """
        Transform a SemanticObject's pose to the robot's base_link.

        This mimics the logic in PrequalGateTask so meta_gate.left/right
        are defined in the same way (left/right in the body frame).
        """
        pose_stamped = PoseStamped()
        pose_stamped.header = obj.header
        pose_stamped.pose = obj.pose.pose

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='base_link',
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
                f"Failed to transform semantic object to base_link"
                f"(current frame: {obj.header.frame_id})"
            )
            self.get_logger().warn(f"exception: {e}")
            return pose_stamped.pose
