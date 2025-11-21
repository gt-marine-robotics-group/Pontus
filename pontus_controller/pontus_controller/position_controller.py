from enum import Enum
import asyncio
import numpy as np
from typing import Optional, List, Dict
import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer
import tf_transformations

from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseStamped
from pontus_msgs.msg import CommandMode
from pontus_msgs.message_enums import CommandModeEnum
from pontus_msgs.action import GoToPose
from rcl_interfaces.msg import SetParametersResult

from pontus_controller.PID import PID

class PositionControllerState(Enum):
    Stopped = 0
    MaintainPosition = 1
    ZCorrection = 2
    FaceTargetPoint = 3
    GoToPoint = 4

class DegreeOfFreedom(Enum):
    X = 0
    Y = 1
    Z = 2
    ROLL = 3
    PITCH = 4
    YAW = 5

class PositionController(Node):
    def __init__(self):
        super().__init__('position_controller')
        self.state = PositionControllerState.Stopped

        param_list = (
            ('default_command_mode', CommandMode.ESTOP),
            ('x_vmax', 0.4), # m/s
            ('y_vmax', 0.2), # m/s
            ('yaw_vmax', 0.35), # radians/s
            ('lookahead_distance', 1.0), # m
            ('x_kp', 1.0),
            ('x_ki', 0.0),
            ('x_kd', 0.0),
            ('y_kp', 0.275),
            ('y_ki', 0.0),
            ('y_kd', 0.0),
            ('z_kp', 0.5),
            ('z_ki', 0.0),
            ('z_kd', 0.0),
            ('r_kp', 0.1),
            ('r_ki', 0.0),
            ('r_kd', 0.0),
            ('p_kp', 0.5),
            ('p_ki', 0.0),
            ('p_kd', 0.0),
            ('yaw_kp', 0.5),
            ('yaw_ki', 0.0),
            ('yaw_kd', 0.0),
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odometry_callback,
            10
        )
        self.command_mode_sub = self.create_subscription(
            CommandMode,
            '/command_mode',
            self.command_mode_callback,
            10
        )
        self.cmd_pos_sub = self.create_subscription(
            Pose,
            '/cmd_pos',
            self.cmd_pos_callback,
            10
        )
        # Rviz/foxglove clicked point subs
        self.rviz_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.rviz_pose_callback,
            10
        )
        self.cmd_vel_manual_sub = self.create_subscription(
          Twist,
          '/cmd_vel',
          self.cmd_vel_callback,
          10)
        self.action_server = ActionServer(
            self,
            GoToPose,
            '/pontus/go_to_pose',
            execute_callback=self.execute_callback,
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel_fused',
            10
        )
        self.debug_pose_pub = self.create_publisher(
            PoseStamped,
            '/debug/pose_command',
            10
        )

        self.prev_time = self.get_clock().now()

        self.add_on_set_parameters_callback(self.param_callback)
        self.declare_parameters(namespace="", parameters=param_list)

        self.pid_linear = [
            PID(self.x_kp, self.x_ki, self.x_kd),
            PID(self.y_kp, self.y_ki, self.y_kd, windup_max=10.0),
            PID(self.z_kp, self.z_ki, self.z_kd, windup_max =1.0)
        ]

        self.pid_angular = [
            PID(self.r_kp, self.r_ki, self.r_kd),
            PID(self.p_kp, self.p_ki, self.p_kd),
            PID(self.yaw_kp, self.yaw_ki, self.yaw_kd, windup_max=2)
        ]

        self.command_mode = self.default_command_mode
        if (self.command_mode != CommandMode.ESTOP):
            self.state = PositionControllerState.MaintainPosition

        self.cmd_pos_linear = np.zeros(3)
        self.cmd_pos_angular = np.zeros(3)
        self.cmd_vel = np.zeros(6)

        self.start_pose = np.zeros(6)

        self.current_pose = Pose()
        self.current_twist = Twist()
        self.previous_state = None

        # TODO: Find a way for these values to be set by a request
        # but easily reset to default

        # Acceptable error for each dof
        self.linear_thresholds = 0.5 # m
        self.depth_threshold = 0.2 # m
        self.angular_thresholds = np.array([0.1, 0.1, 0.1]) # r, p, y
        self.velocity_thresholds = np.array([0.1, 0.3]) # linear, angular

        self.skip_orientation = False

    def command_mode_callback(self, msg: CommandMode) -> None:
        if self.command_mode == msg.command_mode:
            return

        self.get_logger().info(f"Changing Command Mode: {CommandModeEnum(msg.command_mode).name}")

        self.command_mode = msg.command_mode
        self.skip_orientation = False

        if (self.command_mode == CommandMode.ESTOP
            or self.command_mode == CommandMode.DIRECT_CONTROL
            or self.command_mode == CommandMode.VELOCITY_CONTROL):
            self.state = PositionControllerState.Stopped
        else:
            self.state = PositionControllerState.MaintainPosition

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.cmd_vel = np.array([
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        ])

        # Bypass the position controller and send commands directly
        if self.command_mode == CommandMode.DIRECT_CONTROL \
            or self.command_mode == CommandMode.VELOCITY_CONTROL:
            self.cmd_vel_pub.publish(msg)

    async def execute_callback(self, goal_handle: any) -> GoToPose.Result:
        request: GoToPose.Goal = goal_handle.request

        self.command_mode_callback(request.command_mode)
        self.cmd_pos_callback(request.desired_pose, request.use_relative_position)
        self.cmd_vel_callback(request.desired_twist)
        self.skip_orientation = request.skip_orientation

        feedback_msg = GoToPose.Feedback()

        while True:
            if self.is_pose_complete():
                break
            feedback_msg.current_state = self.state.value
            goal_handle.publish_feedback(feedback_msg)
            # This might spam the feedback message pretty fast
            await asyncio.sleep(0)

        goal_handle.succeed()
        result = GoToPose.Result()
        result.completed = True
        return result

    def cmd_pos_callback(self, msg: Pose, use_relative_pos: bool = False,
                         header: Header = None) -> None:
        """
        Stores the commanded pose and configures the position controller

        Args:
        ----
        msg (Pose): The new commanded pose
        use_relative_pos (bool): Whether or not the controller should move to the commanded position
            in global space or relative to the current position of the vehicle
        header (Header): header message including frame and timestamp

        Return:
        ------
        None

        """
        quat = msg.orientation
        (r, p, y) = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        cmd_pos_new = np.array([msg.position.x, msg.position.y, msg.position.z])
        cmd_ang_new = np.array([r, p, y])

        if use_relative_pos and self.current_pose is not None:
            cmd_pos_new += np.array([
                self.current_pose.position.x,
                self.current_pose.position.y,
                self.current_pose.position.z,
            ])

        # If the cmd_pos and cmd_ang is the same as the previous, do nothing
        if np.array_equal(cmd_pos_new, self.cmd_pos_linear) \
                and np.array_equal(cmd_ang_new, self.cmd_pos_angular):
            return

        start_quat = msg.orientation
        (start_r, start_p, start_y) = tf_transformations.euler_from_quaternion([
            start_quat.x,
            start_quat.y,
            start_quat.z,
            start_quat.w
        ])
        self.start_pose = [
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_pose.position.z,
            start_r,
            start_p,
            start_y
        ]

        self.cmd_pos_linear = cmd_pos_new
        self.cmd_pos_angular = cmd_ang_new

        debug_msg = PoseStamped()
        if header is not None:
            debug_msg.header = header
        else:
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = "map"
        debug_msg.pose = msg
        self.debug_pose_pub.publish(debug_msg)

    def rviz_pose_callback(self, msg: PoseStamped):
        # TODO: technically the command may not be in map frame,
        #       consider transforming the pose
        self.cmd_pos_callback(msg.pose, header=msg.header)

    def state_debugger(self) -> None:
        if self.previous_state != self.state:
            self.get_logger().info(f"Now at: {self.state.name}")
            self.previous_state = self.state

    def is_pose_complete(self) -> bool:
        """
        Determines if the position controller has reached the current commanded position

        Args:
        ----
        None

        Return:
        ------
        bool: Whether or not the commanded position has been reached
        """
        pose_array = self.get_current_pose_array()

        goal_linear_err = self.calculate_linear_error(self.cmd_pos_linear, pose_array[0:3])
        goal_angular_err = self.calculate_angular_error(self.cmd_pos_angular, pose_array[3:6])
        dist_to_goal = np.linalg.norm(goal_linear_err[0:2])

        linear_vel = np.array([
            self.current_twist.linear.x,
            self.current_twist.linear.y,
            self.current_twist.linear.z,
        ])
        angular_vel = np.array([
            self.current_twist.angular.x,
            self.current_twist.angular.y,
            self.current_twist.angular.z,
        ])

        orientation_good = self.skip_orientation or np.all(goal_angular_err < self.angular_thresholds)
        velocity_good = np.all(linear_vel < self.velocity_thresholds[0]) \
                           and np.all(angular_vel < self.velocity_thresholds[1])

        if dist_to_goal < self.linear_thresholds \
            and orientation_good \
            and velocity_good:
            return True
        else:
            return False

    def odometry_callback(self, msg: Odometry) -> None:
        """
        Runs the position controller state machine to determine
        and publish the desired body velocities

        Args:
        ----
        msg (Odometry): our current odometry from robot localization

        Return:
        ------
        None

        """
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist
        pose_array = self.get_current_pose_array()

        self.state = self.calculate_state(pose_array)
        self.state_debugger()

        state_target_linear = pose_array[0:3].copy()
        state_target_angular = pose_array[3:6].copy()
        match self.state:
            case PositionControllerState.Stopped:
                return
            case PositionControllerState.MaintainPosition:
                state_target_linear = self.cmd_pos_linear
                state_target_angular = state_target_angular if self.skip_orientation else self.cmd_pos_angular
            case PositionControllerState.ZCorrection:
                state_target_linear = self.cmd_pos_linear[2]
            case PositionControllerState.FaceTargetPoint:
                state_target_linear[2] = self.cmd_pos_linear[2]
                state_target_angular[2] = self.calculate_angle_to_target(self.cmd_pos_linear, pose_array[0:3])
            case PositionControllerState.GoToPoint:
                state_target_linear, state_target_angular = self.calculate_go_to_point_target(pose_array)

        linear_err = self.calculate_linear_error(state_target_linear, pose_array[0:3])
        angular_err = self.calculate_angular_error(state_target_angular, pose_array[3:6])

        dt = self.get_clock().now() - self.prev_time
        self.prev_time = self.get_clock().now()

        commands = np.array([
            self.pid_linear[0](linear_err[0], dt),
            self.pid_linear[1](linear_err[1], dt),
            self.pid_linear[2](linear_err[2], dt),
            self.pid_angular[0](angular_err[0], dt),
            self.pid_angular[1](angular_err[1], dt),
            self.pid_angular[2](angular_err[2], dt)
        ])

        msg = self.generate_command_msg(commands, pose_array)

        if self.command_mode != CommandMode.DIRECT_CONTROL \
            and self.command_mode != CommandMode.VELOCITY_CONTROL:
            self.cmd_vel_pub.publish(msg)

    def calculate_state(self, pose_array: np.ndarray) -> PositionControllerState:
        """
        Determines what state machine state the position controller should be in
        based on the command mode and current position/velocity of the sub

        Args:
        ----
        pose_array (np.ndarray): our current pose in [x, y, z, r, p, yaw]

        Return State: State machine state
        ------

        """

        if self.command_mode == CommandMode.ESTOP \
            or self.command_mode == CommandMode.DIRECT_CONTROL:
            return PositionControllerState.Stopped
        elif self.command_mode == CommandMode.VELOCITY_HOLD_DEPTH \
            or self.command_mode == CommandMode.VELOCITY_HOLD_DEPTH_HEADING \
            or self.command_mode == CommandMode.VELOCITY_HOLD_POSITION:
            return PositionControllerState.MaintainPosition
        elif self.command_mode == CommandMode.POSITION_WITH_STRAFE:
            return PositionControllerState.MaintainPosition

        linear_vel = np.array([
            self.current_twist.linear.x,
            self.current_twist.linear.y,
            self.current_twist.linear.z,
        ])
        angular_vel = np.array([
            self.current_twist.angular.x,
            self.current_twist.angular.y,
            self.current_twist.angular.z,
        ])

        goal_linear_err = self.calculate_linear_error(self.cmd_pos_linear, pose_array[0:3])

        if goal_linear_err[2] > self.depth_threshold:
            return PositionControllerState.ZCorrection

        # Close enough to strafe directly to target
        dist_to_goal = np.linalg.norm(goal_linear_err[0:2])
        if self.command_mode == CommandMode.POSITION_WITH_STRAFE \
            or dist_to_goal < 2.0 * self.linear_thresholds:

            return PositionControllerState.MaintainPosition

        # Remaining states need to point towards the target point
        # instead of the final angle
        angle_to_target_point = np.array([0, 0, self.calculate_angle_to_target(self.cmd_pos_linear, pose_array[0:3])])
        face_target_angular_err = self.calculate_angular_error(angle_to_target_point, pose_array[3:6])

        if abs(face_target_angular_err[2]) > self.angular_thresholds[2] \
            or abs(angular_vel[2]) > 0.1:

            return PositionControllerState.FaceTargetPoint

        # Need to be facing target point and have yaw velocity settled before reaching this state
        return PositionControllerState.GoToPoint

    def generate_command_msg(self, commands: np.ndarray, pose_array: np.ndarray) -> Twist:
        """
        Handles the various velocity hold modes that control only specific
        degrees of freedom, applys the max velocity limits, then builds the commanded twist message

        Args:
        ----
        commands (np.ndarray): array of velocity commands for the degrees of freedom: [x, y, z, r, p, yaw]
        pose_array (np.ndarray): our current pose in [x, y, z, r, p, yaw]

        Return Twist: Desired body velocity twist message
        ------

        """

        if self.command_mode == CommandMode.VELOCITY_HOLD_DEPTH \
            or self.command_mode == CommandMode.VELOCITY_HOLD_DEPTH_HEADING \
            or self.command_mode == CommandMode.VELOCITY_HOLD_POSITION:

            # Figure out which degrees of freedom to allow the controller to influence
            # and override the request position if they are changed by the incoming velocity command
            command_indices: List[int] = []
            if self.command_mode == CommandMode.VELOCITY_HOLD_DEPTH:
                command_indices = [DegreeOfFreedom.Z.value]
            elif self.command_mode == CommandMode.VELOCITY_HOLD_DEPTH_HEADING:
                command_indices = [DegreeOfFreedom.Z.value, DegreeOfFreedom.YAW.value]
            elif self.command_mode == CommandMode.VELOCITY_HOLD_POSITION:
                command_indices = list(range(len(commands)))

            for i in range(len(commands)):
                if i in command_indices:
                    if abs(self.cmd_vel[i]) > 0.01:
                        commands[i] = self.cmd_vel[i]

                        if i < 3:
                            self.cmd_pos_linear[i] = pose_array[i]
                        else:
                            self.cmd_pos_angular[i%3] = pose_array[i]
                else:
                    commands[i] = self.cmd_vel[i]

        msg = Twist()
        msg.linear.x = commands[0]
        msg.linear.y = commands[1]
        msg.linear.z = commands[2]
        msg.angular.x = commands[3]
        msg.angular.y = commands[4]
        msg.angular.z = commands[5]

        # TODO: This may cause issues while moving forward 
        # and strafing where it clamps different axes seperately so the final movement
        # does not follow the same angle
        if abs(msg.linear.x) > self.x_vmax:
            msg.linear.x = np.sign(msg.linear.x) * self.x_vmax
        if abs(msg.linear.y) > self.y_vmax:
            msg.linear.y = np.sign(msg.linear.y) * self.y_vmax
        if abs(msg.angular.z) > self.yaw_vmax:
            msg.angular.z = np.sign(msg.angular.z) * self.yaw_vmax
        return msg

    def get_current_pose_array(self):
        """
        Convert the most recently received odometry message into a pose array

        Args:
        ----
        None

        Return np.ndarray: our current pose in [x, y, z, r, p, yaw]
        ------

        """
        (r, p, yaw) = tf_transformations.euler_from_quaternion([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w,
        ])

        return np.array([
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_pose.position.z,
            r,
            p,
            yaw
        ])

    def calculate_angle_to_target(self, 
                                  target: np.ndarray,
                                  current_pose: np.ndarray) -> np.ndarray:
        """
        Calculate the yaw angle to point from current pose to the target point

        Args:
        ----
        target (np.ndarray): the target point [x, y, z]
        current_pose (np.ndarray): the point we are currently at [x, y, z]

        Return float: Yaw angle in radians
        ------

        """

        linear_difference = target - current_pose
        orientation_to_target = np.arctan2(linear_difference[1], linear_difference[0])
        return orientation_to_target

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
                               current_pose: np.ndarray) -> np.ndarray:
        """
        Calculate the linear error between our desired pose and our current pose.

        Args:
        ----
        desired_pose (np.ndarray): our desired position in [x, y, z]
        current_pose (np.ndarray): our current position in [x, y, z]

        Return:
        ------
        np.ndarray: our linear error in [x, y, z]

        """
        u = np.array([
            self.current_pose.orientation.x,
            -self.current_pose.orientation.y,
            -self.current_pose.orientation.z,
        ])
        s = self.current_pose.orientation.w
        linear_err = desired_pose - current_pose
        linear_err = (2.0 * np.dot(u, linear_err) * u
                      + (s**2 - np.dot(u, u)) * linear_err
                      + 2.0 * s * np.cross(u, linear_err))
        return linear_err

    def calculate_go_to_point_target(self,
                  pose_array: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Uses a lookahead distance to calculate a target point 
        to go to on the line between the starting point of the vehicle 
        when it received the commanded position and its commanded position.
        This keeps it from drifting sideways or driving in curves

        Args:
        ----
        pose_array (np.ndarray): our current pose in [x, y, z, r, p, yaw]

        Return:
        ------
        np.ndarray: our linear error in [x, y, z]
        np.ndarray: our angular error in [r, p, y]

        """

        trajectory_vector = self.cmd_pos_linear[:2] - self.start_pose[:2]
        trajectory_unit = trajectory_vector / np.linalg.norm(trajectory_vector)

        vec = pose_array[:2] - self.start_pose[:2]

        proj_length = np.dot(vec, trajectory_unit)
        trajectory_length = np.linalg.norm(trajectory_vector)
        proj_length = max(0.0, min(proj_length, trajectory_length - self.lookahead_distance))
        projected_point = self.start_pose[:2] + proj_length * trajectory_unit

        target_point = projected_point + self.lookahead_distance * trajectory_unit
        target_point = np.append(target_point, self.cmd_pos_linear[2])

        linear_difference = self.cmd_pos_linear - pose_array[0:3]
        orientation_to_target = np.array([0, 0, np.arctan2(linear_difference[1], linear_difference[0])])

        return target_point, orientation_to_target

    def param_callback(self, params: Dict[str, any]) -> SetParametersResult:
        """
        Handles ROS2 param changes to update the nodes member variables as well
        as the internal PID parameters

        Args:
        ----
        params (dict): A dictionary of parameter values to set on the class

        Return:
        ------
        SetParametersResult

        """
        for param in params:
            setattr(self, param.name, param.value)

            if hasattr(self, "pid_linear") and hasattr(self, "pid_angular"):
                split = param.name.split("_")
                dof = split[0]
                gain = split[1]

                pid_obj = None
                match dof:
                    case "x":
                        pid_obj = self.pid_linear[0]
                    case "y":
                        pid_obj = self.pid_linear[1]
                    case "z":
                        pid_obj = self.pid_linear[2]
                    case "r":
                        pid_obj = self.pid_angular[0]
                    case "p":
                        pid_obj = self.pid_angular[1]
                    case "yaw":
                        pid_obj = self.pid_angular[2]

                if (pid_obj is not None):
                    setattr(pid_obj, gain, param.value)

        return SetParametersResult(successful=True)

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PositionController()
    rclpy.spin(node)