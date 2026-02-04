# ------ Libraries ------
import rclpy

from enum import Enum
from math import fabs
import tf_transformations

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from vision_msgs.msg import Detection2DArray

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj
from pontus_controller.position_controller import MovementMethod

from pontus_msgs.msg import SemanticMap


class SearchConditions(Enum):
    GATE = 0
    SLALOM = 1
    TARGET = 2
    BIN = 3
    OCTAGON = 4
    FULL_SCAN = 5  # After Slalom try to find all remaining tasks


class SearchTask(BaseTask):

    def __init__(self,
                 target_angle1_rad: float,
                 target_angle2_rad: float,
                 terminating_condition: SearchConditions,
                 angle_tolerance: float = 0.1,
                 state_machine_clock_s: float = 0.1,
                 desired_depth: float = -1.2) -> None:

        super().__init__()

        # --- Params ---
        self.target_angle1_rad = target_angle1_rad
        self.target_angle2_rad = target_angle2_rad
        self.terminating_condition = terminating_condition
        self.angle_tolerance = angle_tolerance
        self.desired_depth = desired_depth

        # --- State ---
        # the active target angle (relative to start)
        self.target_angle = None
        self.finished_first_turn = False
        self.start_angle = None             # absolute yaw at start (rad)
        # absolute yaw updated from odometry (rad)
        self.current_angle = None

        # --- Helpers ---
        self.go_to_pose_client = GoToPoseClient(self)

        # --- Subscriptions ---
        self.semantic_map = None
        self.semantic_map_sub = self.create_subscription(
            SemanticMap,
            '/pontus/semantic_map',
            self.semantic_map_sub_callback,
            10
        )

        self.yolo_detections = None
        self.yolo_detections_sub = self.create_subscription(
            Detection2DArray,
            '/pontus/camera_front/yolo_results',
            self.yolo_detection_sub_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/pontus/odom',
            self.odometry_sub_callback,
            10
        )

        self.state_machine_timer = self.create_timer(
            state_machine_clock_s, self.state_machine
        )

    def odometry_sub_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )
        self.current_angle = yaw
        if self.start_angle is None:
            self.start_angle = yaw

    def semantic_map_sub_callback(self, msg: SemanticMap) -> None:
        self.semantic_map = msg

    def yolo_detection_sub_callback(self, msg: Detection2DArray) -> None:
        self.yolo_detections = msg

    # ------ FSM ------
    def state_machine(self) -> None:
        # 0) Wait until we know our heading
        if self.current_angle is None or self.start_angle is None:
            return

        # 1) success condition
        if self._are_conditions_met():
            self._finish_task(success=True)

        # 2) send command if not done yet
        if self.target_angle is None:
            self.target_angle = self.target_angle1_rad
            self._send_turn_command(self.target_angle)
            return

        # 3) check if we have arrived at first angle
        if self._at_target_angle(self.target_angle):
            if self.target_angle == self.target_angle1_rad:
                self.target_angle = self.target_angle2_rad
                self._send_turn_command(self.target_angle)
            else:
                self._finish_task(self, success=False)

    def _at_target_angle(self, rel_target_angle: float) -> bool:
        """
        rel_target_angle is relative to start; convert to absolute
        and compare with current yaw.
        """
        abs_target = self.start_angle + rel_target_angle
        diff = self._angle_diff(self.current_angle, abs_target)
        return fabs(diff) <= self.angle_tolerance

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Smallest signed difference a-b wrapped to [-pi, pi]."""
        import math
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d

    def _are_conditions_met(self) -> bool:
        match self.terminating_condition:

            case SearchConditions.GATE:
                if self.semantic_map is None:
                    return False

                return (
                    len(self.semantic_map.gate_left) >= 1 and
                    len(self.semantic_map.gate_right) >= 1
                    # and _is_detected("gate_image_fish") and
                    # _is_detected("gate_image_shark")
                )

            case SearchConditions.SLALOM:
                if self.semantic_map is None:
                    return False

                #TODO: more advance logic 

                return (
                    len(self.semantic_map.slalom_red) >= 1 and
                    len(self.semantic_map.slalom_white) >= 2
                )

            case SearchConditions.TARGET:
                if self.semantic_map is None:
                    return False

                return (
                    len(self.semantic_map.target) >= 1
                    # _is_detected("target")
                )

            case SearchConditions.BIN:
                if self.semantic_map is None:
                    return False

                return (
                    len(self.semantic_map.bin) >= 1
                    # _is_detected("bin")
                )

            case SearchConditions.OCTAGON:
                if self.semantic_map is None:
                    return False

                return (
                    len(self.semantic_map.octagon) >= 1
                    # _is_detected("octagon")
                )

            case SearchConditions.FULL_SCAN:
                # If we want to try to find all remaining tasks after slalom
                if self.semantic_map is None:
                    return False

                return (
                    len(self.semantic_map.bin) >= 1 and
                    len(self.semantic_map.target) >= 1 and
                    len(self.semantic_map.octagon) >= 1
                )

            case _:
                self.get_logger().warn("Invalid SearchCondition")
                return True

    def _is_detected(self, label: str) -> bool:

        # for detection in self.yolo_detections:
        #     if detection.results.id ==

        # TODO: Add enum relating detection id to label
        return False

    def _send_turn_command(self, target_angle_rad: float) -> None:
        if self.start_angle is None:
            return

        cmd_pose = Pose()
        yaw_abs = self.start_angle + target_angle_rad
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(
            0.0, 0.0, yaw_abs)

        # leave x/y to controller or keep current position if supported
        cmd_pose.position.x = 0.0
        cmd_pose.position.y = 0.0
        cmd_pose.position.z = self.desired_depth
        cmd_pose.orientation.x = qx
        cmd_pose.orientation.y = qy
        cmd_pose.orientation.z = qz
        cmd_pose.orientation.w = qw

        command = PoseObj(
            cmd_pose=cmd_pose,
            skip_orientation=False,
            movement_method=MovementMethod.TurnThenForward
        )
        self.go_to_pose_client.go_to_pose(command)

    def _finish_task(self, success: bool) -> bool:
        self.get_logger().info("Finished the search task")
        self.complete()
