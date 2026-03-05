##
# Abstracted
# go_to_pose
# current_state
# at_pose

import rclpy
from rclpy.action import ActionClient
from rclpy.task import Future
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Path

from pontus_msgs.msg import CommandMode
from pontus_msgs.action import GoToPose
from pontus_msgs.srv import GetPathToObject
from pontus_controller.position_controller import PositionControllerState
from pontus_autonomy.base_run import BaseTask


class PoseObj:
    def __init__(self,
                 cmd_pose: Pose = None,
                 cmd_twist: Twist = None,
                 skip_orientation: bool = False,
                 use_relative_position: bool = False,
                 use_path_planning: bool = True,
                 command_mode: int = CommandMode.POSITION_FACE_TRAVEL):

        self.cmd_pose = cmd_pose
        self.cmd_twist = cmd_twist

        self.command_mode = command_mode
        self.skip_orientation = skip_orientation
        self.use_relative_position = use_relative_position
        self.use_path_planning = use_path_planning


class GoToPoseClient:
    def __init__(self, node: BaseTask):
        self.action_client = ActionClient(
            node,
            GoToPose,
            '/pontus/go_to_pose',
        )

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            node.get_logger().error('GoToPose action server not available.')
        self.current_state = PositionControllerState.MaintainPosition
        self.completed = False
        self.is_in_progress = False
        self.node = node

    # Ros architecture
    def go_to_pose(self, pose_obj: PoseObj) -> None:
        """
        Start Action client to go to pose.

        Args:
        ----
        pose_obj (PoseObj): the desired input for the controller

        Return:
        ------
        None

        """
        self.completed = False
        self.is_in_progress = True
        goal_msg: GoToPose.Goal = GoToPose.Goal()
        goal_msg.skip_orientation = pose_obj.skip_orientation
        goal_msg.use_relative_position  = pose_obj.use_relative_position
        goal_msg.command_mode.command_mode = pose_obj.command_mode

        if pose_obj.cmd_pose is not None:
            goal_msg.desired_pose = pose_obj.cmd_pose
        if pose_obj.cmd_twist is not None:
            goal_msg.desired_twist = pose_obj.cmd_twist

        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback: GoToPose.Feedback) -> None:
        """
        Handle the feedback from the gotopose callback.

        Args:
        ----
        feedback (GoToPose.Feedback): feedback object from gotopose

        Return:
        ------
        None

        """
        self.current_state = PositionControllerState(feedback.feedback.current_state)

    def goal_response_callback(self, future: Future) -> None:
        """
        Handle the goal response callback.

        Args:
        ----
        future (Future): the send goal future

        Return:
        ------
        None

        """
        goal_handle = future.result()
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
 
    def get_result_callback(self, future: Future) -> None:
        """
        Handle the result callback future from the action server.

        Args:
        ----
        future (Future): the result future

        Return:
        ------
        None

        """
        result = future.result().result
        completed = result.completed
        # Idk what to do here
        if not completed:
            pass
        self.completed = True

    # Abstraction Layer
    def get_current_state(self) -> PositionControllerState:
        """
        Return what state the position controller is in.

        Args:
        ----
        None

        Return:
        ------
        State: the current state the position controller is in

        """
        return self.current_state

    def at_pose(self) -> bool:
        """
        Return whether or not the AUV is at the pose.

        Args:
        ----
        None

        Return:
        ------
        bool: true if completed

        """
        return self.completed

    def in_progress(self) -> bool:
        """
        Return whether or not the action is still in progress.

        Args:
        ----
        None

        Return:
        ------
        bool: true if in progress else false

        """
        return self.is_in_progress
    
    # def _send_path_planning_request(self, goal_pose: PoseStamped) -> Path:
    #     request = GetPathToObject.Request()
    #     request.goal = goal_pose
    #     future = self.path_planner_client.call_async(request)
    #     rclpy.spin_until_future_complete(self.node, future)
    #     return future.result()
        