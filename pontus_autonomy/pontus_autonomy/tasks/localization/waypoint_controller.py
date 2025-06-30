from enum import Enum
import numpy as np

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
import tf_transformations

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj
from pontus_controller.position_controller import MovementMethod
from pontus_msgs.msg import YOLOResultArray, YOLOResultArrayPose
from pontus_mapping.semantic_map_manager import SemanticObject


class WaypointContoller(BaseTask):
    class State(Enum):
        WayPoint = 0
        Done = 1

    def __init__(self):
        super().__init__('vertical_task_prequal_velocity')

        self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odometry_callback,
            10
        )

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
        # 14.5 0.5
        vm_x = 13.5
        vm_y = 0.0
        default_z = -1.4
        desired_positions = []
        pose_0 = Pose()
        pose_0.position.x = 0.0
        pose_0.position.y = 0.0
        pose_0.position.z = default_z
        desired_positions.append(pose_0)
        pose_1 = Pose()
        pose_1.position.x = vm_x - 1.0
        pose_1.position.y = vm_y
        pose_1.position.z = default_z
        desired_positions.append(pose_1)
        pose_2 = Pose()
        pose_2.position.x = vm_x - 1.0
        pose_2.position.y = vm_y + 0.7
        pose_2.position.z = default_z
        desired_positions.append(pose_2)
        pose_3 = Pose()
        pose_3.position.x = vm_x + 1.0
        pose_3.position.y = vm_y + 0.7
        pose_3.position.z = default_z
        desired_positions.append(pose_3)
        pose_4 = Pose()
        pose_4.position.x = vm_x + 1.0
        pose_4.position.y = vm_y - 1.0
        pose_4.position.z = default_z
        desired_positions.append(pose_4)
        pose_5 = Pose()
        pose_5.position.x = vm_x - 1.0
        pose_5.position.y = vm_y - 1.0
        pose_5.position.z = default_z
        desired_positions.append(pose_5)
        pose_6 = Pose()
        pose_6.position.x = 0.0
        pose_6.position.y = 0.0
        pose_6.position.z = default_z
        desired_positions.append(pose_6)

        if not self.command_sent:
            cmd_pose = desired_positions[self.current_desired_position]
            self.command_sent = True
            self.get_logger().info(f"{desired_positions[self.current_desired_position]}")
            if self.current_desired_position == 0 or self.current_desired_position == 1 or self.current_desired_position == 3 or self.current_desired_position == 5 or self.current_desired_position == 6:
                return PoseObj(cmd_pose=cmd_pose,
                               skip_orientation=True,
                               desired_depth=-1.5,
                               movement_method=MovementMethod.TurnThenForward)
            else:
                return PoseObj(cmd_pose=cmd_pose,
                               skip_orientation=True,
                               desired_depth=-1.5,
                               movement_method=MovementMethod.StrafeThenForward)

        elif self.go_to_pose_client.at_pose():
            self.current_desired_position += 1
            self.command_sent = False

        if self.current_desired_position == len(desired_positions):
            self.state = self.State.Done

        return None
