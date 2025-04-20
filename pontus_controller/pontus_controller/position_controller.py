import rclpy
from rclpy.node import Node
import numpy as np
from enum import Enum
from typing import Any

from rclpy.action import ActionServer
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Bool
import asyncio

from .PID import PID
from pontus_msgs.action import GoToPose
from typing import Optional, List


class PositionControllerState(Enum):
    Maintain_position = 1
    Z_correction = 2
    Strafe = 3
    Direction_correction = 4
    Linear_correction = 5
    Angular_correction = 6
    Velocity_pass_through = 7
    Velocity_maintain_depth = 8
    Velocity_maintain_depth_heading = 9


class MovementMethod(Enum):
    TurnThenForward = 1
    StrafeThenForward = 2
    Velocity = 3
    VelocityMaintainDepth = 4
    VelocityMaintainDepthHeading = 5


class PositionNode(Node):
    def __init__(self):
        super().__init__('position_controller')

        self.state = PositionControllerState.Maintain_position
        self.transition_threshold = {}
        self.transition_threshold[PositionControllerState.Z_correction] = 0.1
        self.transition_threshold[PositionControllerState.Direction_correction] = 0.1
        self.transition_threshold[PositionControllerState.Linear_correction] = 0.1
        self.transition_threshold[PositionControllerState.Strafe] = 0.05
        self.transition_threshold[PositionControllerState.Angular_correction] = 0.10

        self.deadzone = 0.2
        self.linear_deadzone = 0.5

        self.cmd_linear = None
        self.cmd_angular = None

        self.prev_time = self.get_clock().now()
        self.prev_linear_time_under_threshold = self.get_clock().now()

        self.declare_parameter('sim_mode', False)
        sim_mode = self.get_parameter('sim_mode').get_parameter_value().bool_value
        # If this is true, stop the position controller from publishing command velocities
        self.controller_mode = True

        # If we are in sim, there are no RC so automatically turn position controller on
        if sim_mode:
            self.controller_mode = False

        # TODO: Tune these
        # Sim PID values
        if sim_mode:
            self.pid_linear = [
                PID(1, 0, 0.7),
                PID(1, 0, 0.5),
                PID(2, 0, 2)
            ]
            self.pid_angular = [
                PID(0.5, 0, 0),
                PID(4, 0, 0),
                PID(0.4, 0, 0)
            ]
        # Real values for sub
        else:
            self.pid_linear = [
                PID(1.0, 0, 0),
                PID(0.5, 0, 0),
                PID(0.5, 0, 0)
            ]

            self.pid_angular = [
                PID(0.1, 0, 0),
                PID(0.5, 0, 0),
                PID(0.15, 0.01, 0.000001, windup_max=10)
            ]
        # PID(0.15, 0.001, 0.000001, 5)
        self.thresh = 0.2
        self.angular_thresh = 0.1
        self.hold_point = False

        self.goal_pose = np.zeros(3)
        self.goal_angle = np.zeros(3)
        self.goal_twist = None
        self.skip_orientation = False
        self.desired_depth = 0
        self.desired_heading = 0
        self.current_pose = None
        self.movement_method = MovementMethod.TurnThenForward
        # ROS infrastructure
        self.cmd_pos_sub = self.create_subscription(
          Pose,
          '/cmd_pos',
          self.cmd_pos_callback,
          10
        )

        self.odom_sub = self.create_subscription(
          Odometry,
          '/pontus/odometry',
          self.odometry_callback,
          10
        )

        self.autonomy_sub = self.create_subscription(
            Bool,
            '/autonomy_mode',
            self.autonomy_mode_callback,
            10
        )

        self.action_server = ActionServer(
            self,
            GoToPose,
            '/pontus/go_to_pose',
            execute_callback=self.execute_callback,
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.hold_point_pub = self.create_publisher(Bool, '/hold_point', 10)

        self.previous_state = None
        self.desired_pose = None

    async def execute_callback(self, goal_handle: Any) -> GoToPose.Result:
        """
        Handle go to pose action.

        This will check the state until we are back at maintain_position. This state indicates
        that the position controller has finished executing.

        Args:
        ----
        goal_handle (Any): the goal handle of the action

        Return:
        ------
        GoToPose.Result: the result of the action

        """
        request = goal_handle.request
        self.cmd_pos_callback(request.desired_pose)
        self.skip_orientation = request.skip_orientation
        self.movement_method = MovementMethod(request.movement_method)
        if self.movement_method == MovementMethod.Velocity:
            self.goal_twist = request.desired_twist
            self.state = PositionControllerState.Velocity_pass_through
        if self.movement_method == MovementMethod.VelocityMaintainDepth:
            self.goal_twist = request.desired_twist
            self.desired_depth = request.desired_depth
            self.state = PositionControllerState.Velocity_maintain_depth
        if self.movement_method == MovementMethod.VelocityMaintainDepthHeading:
            self.goal_twist = request.desired_twist
            self.desired_depth = request.desired_depth
            self.desired_heading = request.desired_heading
            self.state = PositionControllerState.Velocity_maintain_depth_heading
        else:
            feedback_msg = GoToPose.Feedback()
            while True:
                if self.state == PositionControllerState.Maintain_position:
                    break
                current_state = self.state.value
                feedback_msg.current_state = current_state
                goal_handle.publish_feedback(feedback_msg)
                # Yield control back to the executor loop
                await asyncio.sleep(0)

        goal_handle.succeed()
        result = GoToPose.Result()
        result.completed = True
        return result

    def autonomy_mode_callback(self, msg: Bool) -> None:
        """
        Keep track of autonomy mode.

        When autonomy mode is turned off, then we turn off the position controller. This allows
        for velocities to be directly published to the velocity controller (RC). When autonomy
        mode is turned back on, then the position controller takes care of publsihing velocities.

        Args;
        ----
        msg (Bool): Bool message from callback

        Return:
        ------
        None

        """
        if msg.data:
            self.controller_mode = False
        else:
            self.controller_mode = True

    def state_debugger(self) -> None:
        """
        Publish state on state changes for debugging purposes.

        Args:
        ----
        None

        Return:
        ------
        None

        """
        # self.get_logger().info(f"Goal pose: {self.goal_pose}")
        if self.previous_state != self.state:
            self.get_logger().info(f"Now at: {self.state.name}")
            self.previous_state = self.state

    def cmd_pos_callback(self, msg: Pose) -> None:
        """
        Store the commanded positions to use in the odometry callback.

        Args:
        ----
        msg (Pose): commanded pose

        Return:
        ------
        None

        """
        quat = msg.orientation
        (r, p, y) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        cmd_pos_new = np.array([msg.position.x, msg.position.y, msg.position.z])
        cmd_ang_new = np.array([r, p, y])
        self.goal_pose = [self.goal_pose[0], self.goal_pose[1], cmd_pos_new[2]]
        # If the cmd_pos and cmd_ang is the same as the previous, do nothing
        if np.array_equal(cmd_pos_new, self.cmd_linear) \
                and np.array_equal(cmd_ang_new, self.cmd_angular):
            return
        # If we get a new cmd_pos or cmd_ang, transition to the Z_correction state
        self.cmd_linear = cmd_pos_new
        self.cmd_angular = cmd_ang_new
        self.state = PositionControllerState.Z_correction

    def angle_wrap(self, angle: float) -> float:
        """
        Ensure that the angle is in the range [-pi, pi].

        Args:
        ----
        angle (float): the desired angle we want to convert

        Return:
        ------
        float: the new angle

        """
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def get_next_state(self,
                       current_state: PositionControllerState,
                       skip_orientation: bool,
                       movement_method: MovementMethod) -> PositionControllerState:
        """
        Return the next state based on the parameters.

        This basically defines the structure of our state machine, which may change based on
        the inputs from the action. Represents the δ function p = δ(q, a).

        Args:
        ----
        current_state (PositionControllerState): represents our current state
        skip_orientation (bool): whether or not to skip fixing our orientation at the end
        movement_method (MovementMethod): represents how the sub should reach the desired pose

        Return:
        ------
        PositionControllerState: the next position controller state in the state machine

        """
        match current_state:
            case PositionControllerState.Z_correction:
                if movement_method == MovementMethod.StrafeThenForward:
                    return PositionControllerState.Strafe
                if movement_method == MovementMethod.TurnThenForward:
                    return PositionControllerState.Direction_correction
            case PositionControllerState.Strafe:
                return PositionControllerState.Direction_correction
            case PositionControllerState.Direction_correction:
                return PositionControllerState.Linear_correction
            case PositionControllerState.Linear_correction:
                if skip_orientation:
                    return PositionControllerState.Maintain_position
                else:
                    return PositionControllerState.Angular_correction
            case PositionControllerState.Angular_correction:
                return PositionControllerState.Maintain_position
        self.get_logger().warn("Encountered unhandled case of movement method " +
                               f"{movement_method} at state {current_state}")

    def odometry_callback(self, msg: Odometry) -> None:
        """
        Take in current odometry to command velocity to reach a pose.

        Args:
        ----
        msg (Odometry): our current odometry

        Return:
        ------
        None

        """
        self.state_debugger()
        self.current_pose = msg.pose.pose
        # # Get the current positions from odometry
        quat = msg.pose.pose.orientation
        quat = [quat.x, quat.y, quat.z, quat.w]
        # transform world frame to body frame
        u = np.array(quat[:3])
        u[1] = -u[1]
        u[2] = -u[2]
        current_position = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     msg.pose.pose.position.z])
        quat = msg.pose.pose.orientation
        quat = [quat.x, quat.y, quat.z, quat.w]
        current_position = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     msg.pose.pose.position.z])
        match self.state:
            case PositionControllerState.Maintain_position:
                linear_err, angular_err = self.maintain_position(current_position, quat)
            case PositionControllerState.Z_correction:
                linear_err, angular_err = self.correct_z(current_position, quat)
            case PositionControllerState.Strafe:
                linear_err, angular_err = self.correct_strafe(current_position, quat)
            case PositionControllerState.Direction_correction:
                linear_err, angular_err = self.turn_to_next_point(current_position, quat)
            case PositionControllerState.Linear_correction:
                linear_err, angular_err = self.go_to_point(current_position, quat)
            case PositionControllerState.Angular_correction:
                linear_err, angular_err = self.correct_orientation(current_position, quat)
            case PositionControllerState.Velocity_pass_through:
                linear_err, angular_err = np.zeros(3), np.zeros(3)
            case PositionControllerState.Velocity_maintain_depth:
                z_error = self.maintain_z(self.desired_depth, current_position)
            case PositionControllerState.Velocity_maintain_depth_heading:
                z_error = self.maintain_z(self.desired_depth, current_position)
                current_angle = euler_from_quaternion(quat)
                angle_error = self.calculate_angular_error(
                    np.array([0, 0, self.desired_heading]),
                    current_angle)[2]
            case _:
                linear_err = np.zeros(3)
                angular_err = np.zeros(3)

        dt = self.get_clock().now() - self.prev_time
        msg: Twist = Twist()
        if self.state == PositionControllerState.Velocity_pass_through:
            msg = self.goal_twist
        elif self.state == PositionControllerState.Velocity_maintain_depth:
            msg = self.goal_twist
            msg.linear.z = self.pid_linear[2](z_error, dt)
        elif self.state == PositionControllerState.Velocity_maintain_depth_heading:
            msg = self.goal_twist
            msg.linear.z = self.pid_linear[2](z_error, dt)
            msg.angular.z = self.pid_angular[2](angle_error, dt)
        else:
            msg.linear.x = self.pid_linear[0](linear_err[0], dt)
            msg.linear.y = self.pid_linear[1](linear_err[1], dt)
            msg.linear.z = self.pid_linear[2](linear_err[2], dt)

            msg.angular.x = self.pid_angular[0](angular_err[0], dt)
            msg.angular.y = self.pid_angular[1](angular_err[1], dt)
            msg.angular.z = self.pid_angular[2](angular_err[2], dt)

        self.prev_time = self.get_clock().now()
        msg = self.clamp_velocity(msg)
        # If we are in controller mode, we are publishing command velociites from the rc controller
        if not self.controller_mode:
            self.cmd_vel_pub.publish(msg)
            self.hold_point_pub.publish(Bool(data=self.hold_point))

    def clamp_velocity(self, msg: Twist) -> Twist:
        """
        Clamp the veloctiy to ensure our sub does not over actuate.

        Args:
        ----
        msg (TWist): the desired twist

        Return:
        ------
        Twist: the clamped twist

        """
        # Roughly 30 degrees / s
        if abs(msg.angular.z) > 0.18:
            msg.angular.z = np.sign(msg.angular.z) * 0.18
        if abs(msg.linear.x) > 0.25:
            msg.linear.x = np.sign(msg.linear.x) * 0.25
        if abs(msg.linear.y) > 0.25:
            msg.linear.y = np.sign(msg.linear.y) * 0.25
        return msg

    def maintain_z(self,
                   desired_depth: float,
                   current_position: np.ndarray) -> float:
        """
        Calculate the error to maintain our depth.

        Args:
        ----
        desired_depth (float): the desired depth we want to maintain
        current_position (np.ndarray): our current position

        Return:
        ------
        float: z error

        """
        return desired_depth - current_position[2]

    def calculate_angular_error(self,
                                desired_angle: np.ndarray,
                                current_angle: np.ndarray) -> np.ndarray:
        """
        Calculate angular error between two angles.

        Args:
        ----
        desired_angle (np.ndarray): the angle we want to go to [r, p ,y]
        current_angle (np.ndarray): the angle we currently are at [r, p, y]

        Return:
        ------
        np.ndarray: the error in [r, p, y]

        """
        angular_diff = desired_angle - current_angle
        # find the shorter turn for angles
        angular_adj = np.sign(angular_diff) * 2 * np.pi
        angular_diff_alt = angular_diff - angular_adj
        angular_err = np.where(np.abs(angular_diff) < np.abs(angular_diff_alt),
                               angular_diff,
                               angular_diff_alt)
        return angular_err

    def calculate_linear_error(self,
                               desired_pose: np.ndarray,
                               current_pose: np.ndarray,
                               quat_orientation: np.ndarray) -> np.ndarray:
        """
        Calculate the linear error between our desired pose and our current pose.

        Args:
        ----
        desired_pose (np.ndarray): our desired position in [x, y, z]
        current_pose (np.ndarray): our current position in [x, y, z]
        quat_orientation (np.ndarray): our current orientation in [x, y, z, w]

        Return:
        ------
        np.ndarray: our linear error in [x, y, z]

        """
        u = np.array(quat_orientation[:3])
        u[1] = -u[1]
        u[2] = -u[2]
        s = quat_orientation[3]
        linear_err = desired_pose - current_pose
        linear_err = (2.0 * np.dot(u, linear_err) * u
                      + (s**2 - np.dot(u, u)) * linear_err
                      + 2.0 * s * np.cross(u, linear_err))
        return linear_err

    def maintain_position(self,
                          current_position: np.ndarray,
                          quat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Maintain our current position.

        Our orientation is needed to convert from odom frame to body frame for thrust vectors.

        Args:
        ----
        current_position (np.ndarray): our current position
        quat (np.ndarray): our current orientation

        Return:
        ------
        tuple[np.ndarray, np.ndarray]: linear error, angular error

        """
        linear_err = self.calculate_linear_error(self.goal_pose, current_position, quat)
        (r, p, y) = euler_from_quaternion(quat)
        current_orientation = np.array([r, p, y])
        angular_err = self.calculate_angular_error(self.goal_angle, current_orientation)

        return linear_err, angular_err

    def correct_z(self,
                  current_position: np.ndarray,
                  quat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Correct our depth.

        Our orientation is needed to convert from odom frame to body frame for thrust vectors.

        Args:
        ----
        current_position (np.ndarray): our current position
        quat (np.ndarray): our current orientation

        Return:
        ------
        tuple[np.ndarray, np.ndarray]: linear error, angular error

        """
        self.goal_pose = [self.goal_pose[0], self.goal_pose[1], self.cmd_linear[2]]
        linear_err = self.calculate_linear_error(self.goal_pose, current_position, quat)
        (r, p, y) = euler_from_quaternion(quat)
        current_orientation = np.array([r, p, y])
        angular_err = self.calculate_angular_error(self.goal_angle, current_orientation)
        transition_thresh = self.transition_threshold[PositionControllerState.Z_correction]
        if abs(self.goal_pose[2] - current_position[2]) < transition_thresh:
            self.state = self.get_next_state(self.state,
                                             self.skip_orientation,
                                             self.movement_method)
        return linear_err, angular_err

    def correct_strafe(self,
                       current_position: np.ndarray,
                       quat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Strafe.

        Our orientation is needed to convert from odom frame to body frame for thrust vectors.

        Args:
        ----
        current_position (np.ndarray): our current position
        quat (np.ndarray): our current orientation

        Return:
        ------
        tuple[np.ndarray, np.ndarray]: linear error, angular error

        """
        linear_err = self.calculate_linear_error(self.cmd_linear, current_position, quat)
        # Remove x error
        linear_err[0] = 0.0
        self.goal_pose = self.cmd_linear
        (r, p, y) = euler_from_quaternion(quat)
        current_orientation = np.array([r, p, y])
        angular_err = self.calculate_angular_error(self.goal_angle, current_orientation)
        transition_thresh = self.transition_threshold[PositionControllerState.Strafe]
        if abs(linear_err[1]) < transition_thresh:
            self.state = self.get_next_state(self.state,
                                             self.skip_orientation,
                                             self.movement_method)
        return linear_err, angular_err

    def turn_to_next_point(self,
                           current_position: np.ndarray,
                           quat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Turn to the next point.

        This is because we are faster going straight.
        Our orientation is needed to convert from odom frame to body frame for thrust vectors.

        Args:
        ----
        current_position (np.ndarray): our current position
        quat (np.ndarray): our current orientation

        Return:
        ------
        tuple[np.ndarray, np.ndarray]: linear error, angular error

        """
        linear_err = self.calculate_linear_error(self.goal_pose, current_position, quat)

        linear_difference = self.cmd_linear - current_position
        if np.linalg.norm(linear_difference[:2]) < self.deadzone:
            self.state = self.get_next_state(self.state,
                                             self.skip_orientation,
                                             self.movement_method)
            return np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])

        (r, p, y) = euler_from_quaternion(quat)
        current_orientation = np.array([r, p, y])
        goal_orientation = np.array([0, 0, np.arctan2(linear_difference[1], linear_difference[0])])
        angular_err = self.calculate_angular_error(goal_orientation, current_orientation)
        transition_thresh = self.transition_threshold[PositionControllerState.Direction_correction]
        if abs(angular_err[2]) < transition_thresh:
            self.state = self.get_next_state(self.state,
                                             self.skip_orientation,
                                             self.movement_method)
            self.goal_angle = goal_orientation
            self.prev_linear_time_under_threshold = self.get_clock().now()
        return linear_err, angular_err

    def go_to_point(self,
                    current_position: np.ndarray,
                    quat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Go forward to the desired point.

        Our orientation is needed to convert from odom frame to body frame for thrust vectors.

        Args:
        ----
        current_position (np.ndarray): our current position
        quat (np.ndarray): our current orientation

        Return:
        ------
        tuple[np.ndarray, np.ndarray]: linear error, angular error

        """
        self.goal_pose = self.cmd_linear
        linear_err = self.calculate_linear_error(self.goal_pose, current_position, quat)
        linear_difference = self.cmd_linear - current_position
        (r, p, y) = euler_from_quaternion(quat)
        current_orientation = np.array([r, p, y])
        goal_orientation = np.array([0, 0, np.arctan2(linear_difference[1], linear_difference[0])])
        angular_err = self.calculate_angular_error(goal_orientation, current_orientation)

        if np.linalg.norm(linear_err[:2]) < self.linear_deadzone:
            angular_err = np.zeros(3)

        transition_thresh = self.transition_threshold[PositionControllerState.Linear_correction]
        if np.linalg.norm(linear_err) < transition_thresh:
            self.state = self.get_next_state(self.state,
                                             self.skip_orientation,
                                             self.movement_method)
            (r, p, y) = euler_from_quaternion(quat)
            current_orientation = np.array([0.0, 0.0, y])
            self.goal_angle = current_orientation

        return linear_err, angular_err

    def correct_orientation(self,
                            current_position: np.ndarray,
                            quat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Correct our orientation.

        Our orientation is needed to convert from odom frame to body frame for thrust vectors.

        Args:
        ----
        current_position (np.ndarray): our current position
        quat (np.ndarray): our current orientation

        Return:
        ------
        tuple[np.ndarray, np.ndarray]: linear error, angular error

        """
        linear_err = self.calculate_linear_error(self.goal_pose, current_position, quat)

        (r, p, y) = euler_from_quaternion(quat)
        current_orientation = np.array([r, p, y])
        angular_err = self.calculate_angular_error(self.cmd_angular, current_orientation)
        transition_thresh = self.transition_threshold[PositionControllerState.Angular_correction]
        if abs(angular_err[2]) < transition_thresh:
            self.goal_angle = self.cmd_angular
            self.state = self.get_next_state(self.state,
                                             self.skip_orientation,
                                             self.movement_method)
        return linear_err, angular_err


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    position_node = PositionNode()
    rclpy.spin(position_node)
    rclpy.shutdown()
