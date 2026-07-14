import numpy as np

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry, Path

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj


class ReturnHomeReplayTask(BaseTask):

    def __init__(self):
        super().__init__("return_home_replay")

        self.service_callback_group = MutuallyExclusiveCallbackGroup()

        # ------ Parameters ------


        self.follow_path_period = 0.25

        self.depth_m = 0.5

        # ------ Variables ------
        self.waypoints_are_created: bool = False
        self.path: Path | None = None
        self.path_future = None
        self.path_index = 0

        self.curr_waypoint: Pose = None

        self.execute_path: bool = False

        self.latest_odom = None

        # ------ ROS Subscriptions ------
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

        self.goal_pose = None

    def set_path(self, path: list[np.ndarray]) -> None:
        """
        Set the return path and allow the task to begin executing.

        A copy is made because follow_path() consumes the list with pop(0).
        """
        validated_path: list[np.ndarray] = []

        for index, waypoint in enumerate(path):
            waypoint_array = np.asarray(waypoint, dtype=float)

            if waypoint_array.shape != (2,):
                raise ValueError(
                    f"Waypoint {index} must have shape (2,), "
                    f"but has shape {waypoint_array.shape}"
                )

            validated_path.append(waypoint_array.copy())

        self.path = validated_path
        self.curr_waypoint = None
        self.execute_path = True

    def odom_callback(self, msg: Odometry) -> None:
        """
        Keep track of current position of the robot
        """

        self.latest_odom = msg

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

    def _send_waypoint_command(self, target_pos_xy: np.ndarray) -> None:
        """
        Convert a np.ndarray 2D vector to a command pose and send to pos_controller
        """
        cmd_pose = Pose()

        cmd_pose.position.x = target_pos_xy[0]
        cmd_pose.position.y = target_pos_xy[1]
        cmd_pose.position.z = -self.depth_m

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
