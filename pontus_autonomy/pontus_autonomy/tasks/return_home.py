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

import tf_transformations


class ReturnHomeTask(BaseTask):

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

        self.semantic_map_sub = self.create_subscription(
            SemanticMap,
            '/pontus/semantic_map',
            self.semantic_map_callback,
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

        self.goal_pose = None

        self.replan_timer = self.create_timer(6.5, self.replan_path_callback)

    def replan_path_callback(self):
        if (not self.execute_path) or (self.goal_pose is None):
            return

        if self.path_future is not None and not self.path_future.done():
            self.get_logger().debug("Path request still in flight; skipping replan tick")
            return

        self.get_logger().info("Replanning path... sending request to path planning service")

        request = GetPathToObject.Request()
        request.goal = self.goal_pose  # PoseStamped

        self.path_future = self.path_planning_client.call_async(request)
        self.path_future.add_done_callback(self._on_path_response)

    # def replan_path_callback(self):
    #     if not self.execute_path or not self.goal_pose:
    #         return

    #     if self.path_future is not None and not self.path_future.done():
    #         self.get_logger().debug("Path request still in flight; skipping replan tick")
    #         return

    #     self.get_logger().info("Replanning path...")
    #     self.get_logger().info("Sending request to path planning service")

    #     request = GetPathToObject.Request()
    #     request.goal = self.goal_pose

    #     # self.execute_path = False

    #     self.path_future = self.path_planning_client.call_async(request)
    #     self.path_future.add_done_callback(self._on_path_response)

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

    def semantic_map_callback(self, msg: SemanticMap) -> None:
        frame_id = getattr(msg.meta_gate.header, "frame_id", "")
        if frame_id == "":
            self.get_logger().debug("Meta Gate not detected")
            return

        if self.execute_path or (self.path is not None and len(self.path.poses) > 0):
            return

        left_pose = msg.meta_gate.left_gate.pose.pose
        right_pose = msg.meta_gate.right_gate.pose.pose

        left_xy = self._pose_to_nparray(left_pose)
        right_xy = self._pose_to_nparray(right_pose)

        goal_xy = 0.5 * (left_xy + right_xy)
        goal_xy[0] -= 1.5

        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(goal_xy[0])
        goal.pose.position.y = float(goal_xy[1])
        goal.pose.position.z = float(-self.pool_depth +
                                     self.height_from_bottom_m)
        goal.pose.orientation.w = 1.0

        self.goal_pose = goal
        self.execute_path = True

        self.get_logger().info(
            f"Goal set from meta_gate in frame '{frame_id}': ({goal_xy[0]:.2f}, {goal_xy[1]:.2f})")
        self.replan_path_callback()  # send the request immediately

    # def semantic_map_callback(self, msg: SemanticMap) -> None:
    #     # If we're already executing a plan, don't rebuild/regenerate.
    #     # (But allow regeneration once the path is empty again.)
    #     if msg.meta_gate.header.frame_id == '':
    #         return

    #     meta_gate_sides = [
    #         self._pose_to_nparray(msg.meta_gate.left_gate.pose),
    #         self._pose_to_nparray(msg.meta_gate.right_gate.pose)
    #     ]

    #     # Generate new waypoints only when we don't currently have a path to execute
    #     if self.latest_odom is not None and not self.path and not self.execute_path:
    #         self.goal_pose = (meta_gate_sides[0] + meta_gate_sides[1]) / 2
    #         self.goal_pose[0] -= 1.0

    #         self.execute_path = True

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

            next_pose = self.path.poses[self.path_index + 1]
            next_pos_xy = np.array([next_pose.pose.position.x,
                                    next_pose.pose.position.y])

            facing_vec = next_pos_xy - target_pos_xy
            facing_vec /= np.linalg.norm(facing_vec)

            facing_angle = np.arctan2(facing_vec[0], facing_vec[1])

            self.path_index += 1

            self.get_logger().info(f"Going to: {target_pos_xy}")
            self._send_waypoint_command(target_pos_xy, facing_angle)

    def _send_waypoint_command(self, target_pos_xy: np.ndarray, facing_angle) -> None:
        """
        Convert a np.ndarray 2D vector to a command pose and send to pos_controller
        """
        cmd_pose = Pose()

        cmd_pose.position.x = target_pos_xy[0]
        cmd_pose.position.y = target_pos_xy[1]
        cmd_pose.position.z = -self.pool_depth + self.height_from_bottom_m

        quat = tf_transformations.quaternion_from_euler(0, 0, facing_angle)
        cmd_pose.orientation.x = quat[0]
        cmd_pose.orientation.y = quat[1]
        cmd_pose.orientation.z = quat[2]
        cmd_pose.orientation.w = quat[3]

        self.curr_waypoint = cmd_pose

        self.go_to_pose_client.go_to_pose(PoseObj(cmd_pose=cmd_pose,
                                                  skip_orientation=False))

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
