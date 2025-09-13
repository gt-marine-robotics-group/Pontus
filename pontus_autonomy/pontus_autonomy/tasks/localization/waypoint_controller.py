from enum import Enum
import numpy as np

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool
import tf_transformations

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj
from pontus_controller.position_controller import MovementMethod
from pontus_msgs.msg import YOLOResultArray, YOLOResultArrayPose
from pontus_mapping.semantic_map_manager import SemanticObject

from pontus_autonomy.helpers.run_info import waypoints_list

AUTONOMY_DEBOUNCE_THRESHOLD = 15


class WaypointContoller(BaseTask):
    class State(Enum):
        WayPoint = 0
        Done = 1

    def __init__(self):
        super().__init__('waypoint_controller')

        self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odometry_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/auto_enable',
            self.auto_enable,
            10
        )

        self.autonomy_switch_count = 0

        self.create_timer(
            0.2,
            self.autonomy
        )

        self.go_to_pose_client = GoToPoseClient(self)
        self.state = self.State.WayPoint
        self.current_odometry = None
        self.sent = False
        self.current_desired_position = 0
        self.previous_state = None
        self.starting_pose = None
        self.command_sent = False

    def odometry_callback(self, msg: Odometry) -> None:
        """
        Handle odometry callback.

        Args:
        ----
        msg (Odoometry): the odometry message

        Return:
        ------
        None

        """
        # if self.current_odometry is None:
        # self.desired_depth = msg.pose.pose.position.z
        self.current_odometry = msg.pose.pose

    def auto_enable(self, msg: Bool) -> None:
        if not msg.data:
            self.autonomy_switch_count += 1
        else:
            self.autonomy_switch_count = 0

        if self.autonomy_switch_count >= AUTONOMY_DEBOUNCE_THRESHOLD:
            self.complete(False)

    def copy_pose(self, pose: Pose) -> Pose:
        """
        Deep copies the given pose.

        Args:
        ----
        pose (Pose): the pose we want to copy

        Return:
        ------
        Pose: the copied pose

        """
        new_pose = Pose()
        new_pose.position.x = pose.position.x
        new_pose.position.y = pose.position.y
        new_pose.position.z = pose.position.z
        new_pose.orientation.x = pose.orientation.x
        new_pose.orientation.y = pose.orientation.y
        new_pose.orientation.z = pose.orientation.z
        new_pose.orientation.w = pose.orientation.w
        return new_pose

    def autonomy(self) -> None:
        """
        Run autonomy state machine.

        Args:
        ----
        None

        Return:
        ------
        None

        """
        pose_obj = None
        match self.state:
            case self.State.WayPoint:
                pose_obj = self.waypoint()
            case self.State.Done:
                self.complete(True)
            case _:
                pass

        if pose_obj is not None:
            self.go_to_pose_client.go_to_pose(pose_obj)

    def waypoint(self) -> PoseObj:
        """
        Circumnavigate around the vertical marker.

        Args:
        ----
        None

        Return:
        ------
        PoseObj: object representing how we should move

        """

        waypoints = waypoints_list

        if self.current_desired_position >= len(waypoints):
            self.state = self.State.Done
            return None

        if not self.command_sent:
            cmd_pose = waypoints[self.current_desired_position]
            self.command_sent = True

            self.get_logger().info(
                f"Waypoint #{self.current_desired_position}: {cmd_pose.cmd_pose.position.x}, {cmd_pose.cmd_pose.position.y}, {cmd_pose.cmd_pose.position.z}")
            if self.current_odometry is not None:
                self.get_logger().info(
                    f"Current pos: # {self.current_odometry.position.x}, {self.current_odometry.position.y}, {self.current_odometry.position.z}")

            return cmd_pose

        elif self.go_to_pose_client.at_pose():
            self.current_desired_position += 1
            self.command_sent = False

        return None
