import numpy as np
from enum import Enum
from dataclasses import dataclass

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.service import Service
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry, Path

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj
from pontus_msgs.srv import GetPathToObject

from pontus_msgs.msg import SemanticMap, SemanticObject


class PathPlanningTestTask(BaseTask):

    def __init__(self):
        super().__init__("path_planning_test_task")

        self.service_callback_group = MutuallyExclusiveCallbackGroup()

        # ------ Parameters ------
        self.declare_parameters(
            namespace='',
            parameters=[
                ('height_from_bottom', 1.0),
                ('follow_path_period', 0.25),
                ('pool_depth', 2.0)
            ]
        )

        self.height_from_bottom_m: float = float(
            self.get_parameter('height_from_bottom').value)
        self.follow_path_period: float = float(
            self.get_parameter('follow_path_period').value)
        self.pool_depth: float = float(self.get_parameter('pool_depth').value)

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

        self.path_planning_client = self.create_client(
            GetPathToObject,
            '/pontus/path_planning_service'
        )

        while not self.path_planning_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # ------ Timers ------
        self.follow_path_timer = self.create_timer(
            self.follow_path_period,
            self.follow_path,
            self.service_callback_group
        )

        self.goal_pose = [35.0, 1.0]
        self.goal_pose = PoseStamped()
        self.goal_pose.pose = Pose()
        self.goal_pose.pose.position.x = 35.0
        self.goal_pose.pose.position.y = 1.0

        self.replan_timer = self.create_timer(6.5, self.replan_path_callback)

    def replan_path_callback(self):
        if self.path_future is not None and not self.path_future.done():
            self.get_logger().debug("Path request still in flight; skipping replan tick")
            return

        self.get_logger().info("Replanning path...")
        self.get_logger().info("Sending request to path planning service")

        request = GetPathToObject.Request()
        request.goal = self.goal_pose

        self.execute_path = False

        self.path_future = self.path_planning_client.call_async(request)
        self.path_future.add_done_callback(self._on_path_response)

    def _on_path_response(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"Path planning service call failed: {e}")
            return

        if resp is None:
            self.get_logger().error("Path planning service returned None")
            return

        self.path = resp.path_to_object
        self.path_index = 0
        self.waypoints_are_created = True
        self.execute_path = True

        self.get_logger().info(
            f"Received path with {len(self.path.poses)} poses")

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

        if self.path is None or len(self.path.poses) == 0:
            return

        if self.curr_waypoint is None or self.go_to_pose_client.at_pose():
            if self.path_index >= len(self.path.poses):
                self.complete(True)
                return

            target_pose = self.path.poses[self.path_index]
            target_pos_xy = np.array([target_pose.pose.position.x,
                                      target_pose.pose.position.y])

            self.path_index += 1

            self.get_logger().info(f"Going to: {target_pos_xy}")
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
