import rclpy
from rclpy.node import Node
from enum import Enum
from pontus_msgs.action import GoToPose
import asyncio
import numpy as np
import tf_transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from pontus_controller.PID import PID
from rclpy.action import ActionServer

class PositionControllerSequence(Enum):
    MaintainPosition = 0
    ZCorrection = 1
    Rotate = 2
    GoToPoint = 3

class LOSController(Node):
    def __init__(self):
        super().__init__('LOS_controller')
        self.sequence = PositionControllerSequence.MaintainPosition
        self.odom_sub = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odometry_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.create_subscription(
            Pose,
            '/cmd_pos',
            self.cmd_pos_callback,
            10
        )
        self.action_server = ActionServer(
            self,
            GoToPose,
            '/pontus/go_to_pose',
            execute_callback=self.execute_callback,
        )

        self.prev_time = self.get_clock().now()
        self.controller_mode = False

        self.pid_linear = [
            PID(1.0, 0, 0),
            PID(0.275, 0.1, 0.00001, windup_max=10),
            PID(0.5, 0, 0)
        ]

        self.pid_angular = [
            PID(0.1, 0, 0),
            PID(0.5, 0, 0),
            PID(0.15, 0.01, 0.000001, windup_max=2)
        ]

        self.cmd_linear = None
        self.cmd_angular = None
        self.goal_pose = np.zeros(3)
        self.goal_angle = np.zeros(3)
        self.start_pose = np.zeros(3)
        self.current_pose = Pose()
        self.previous_state = None

    async def execute_callback(self, goal_handle):
        request = goal_handle.request
        self.cmd_pos_callback(request.desired_pose)
        feedback_msg = GoToPose.Feedback()
        while True:
            if self.sequence == PositionControllerSequence.MaintainPosition:
                break
            current_state = self.sequence.value
            feedback_msg.current_state = current_state
            goal_handle.publish_feedback(feedback_msg)
            await asyncio.sleep(0)
        goal_handle.succeed()
        result = GoToPose.Result()
        result.completed = True
        return result

    def cmd_pos_callback(self, msg):
        quat = msg.orientation
        (r, p, y) = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        cmd_pos_new = np.array([msg.position.x, msg.position.y, msg.position.z])
        cmd_ang_new = np.array([r, p, y])
        self.goal_pose = [self.goal_pose[0], self.goal_pose[1], cmd_pos_new[2]]
        # If the cmd_pos and cmd_ang is the same as the previous, do nothing
        if np.array_equal(cmd_pos_new, self.cmd_linear) \
                and np.array_equal(cmd_ang_new, self.cmd_angular):
            return
        self.start_pose = [self.current_pose.position.x,
                    self.current_pose.position.y,
                    self.current_pose.position.z]
        # If we get a new cmd_pos or cmd_ang, transition to the Z_correction state
        self.cmd_linear = cmd_pos_new
        self.cmd_angular = cmd_ang_new
        self.sequence = PositionControllerSequence.ZCorrection

    def autonomy_mode_callback(self, msg):
        if msg.data:
            self.controller_mode = False
        else:
            self.controller_mode = True
    
    def state_debugger(self):
        if self.previous_state != self.sequence:
            self.get_logger().info(f"Now at: {self.sequence.name}")
            self.previous_state = self.sequence
    
    def odometry_callback(self, msg: Odometry):
        self.state_debugger()
        self.current_pose = msg.pose.pose
        quat = msg.pose.pose.orientation
        quat = [quat.x, quat.y, quat.z, quat.w]
        u = np.array(quat[:3])
        u[1] = -u[1]
        u[2] = -u[2]
        current_position = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     msg.pose.pose.position.z])
        match self.sequence:
            case PositionControllerSequence.MaintainPosition:
                linear_err, angular_err = self.maintain_position(current_position, quat)
            case PositionControllerSequence.ZCorrection:
                linear_err, angular_err = self.correct_z(current_position, quat)
            case PositionControllerSequence.Rotate:
                linear_err, angular_err = self.rotate(current_position, quat)
            case PositionControllerSequence.GoToPoint:
                linear_err, angular_err = self.go_to_point(current_position, quat)
        dt = self.get_clock().now() - self.prev_time
        self.prev_time = self.get_clock().now()
        msg = Twist()
        msg.linear.x = self.pid_linear[0](linear_err[0], dt)
        msg.linear.y = self.pid_linear[1](linear_err[1], dt)
        msg.linear.z = self.pid_linear[2](linear_err[2], dt)

        msg.angular.x = self.pid_angular[0](angular_err[0], dt)
        msg.angular.y = self.pid_angular[1](angular_err[1], dt)
        msg.angular.z = self.pid_angular[2](angular_err[2], dt)
        msg = self.clamp_velocity(msg)
        # If we are in controller mode, we are publishing command velociites from the rc controller
        if not self.controller_mode:
            self.cmd_vel_pub.publish(msg)

    def clamp_velocity(self, msg: Twist) -> Twist:
        # Roughly 30 degrees / s
        if abs(msg.angular.z) > 0.18:
            msg.angular.z = np.sign(msg.angular.z) * 0.18
        if abs(msg.linear.x) > 0.25:
            msg.linear.x = np.sign(msg.linear.x) * 0.25
        if abs(msg.linear.y) > 0.3:
            msg.linear.y = np.sign(msg.linear.y) * 0.3
        return msg

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
        (r, p, y) = tf_transformations.euler_from_quaternion(quat)
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
        (r, p, y) = tf_transformations.euler_from_quaternion(quat)
        current_orientation = np.array([r, p, y])
        angular_err = self.calculate_angular_error(self.goal_angle, current_orientation)
        if abs(self.goal_pose[2] - current_position[2]) < 0.2:
            self.sequence = PositionControllerSequence.Rotate
        return linear_err, angular_err

    def rotate(self, current_position, quat):
        linear_err = self.calculate_linear_error(self.goal_pose, current_position, quat)
        # Rotate to the next point
        linear_difference = self.cmd_linear - current_position
        if np.linalg.norm(linear_difference[:2]) < 0.3:
            self.sequence = PositionControllerSequence.GoToPoint
            return np.zeros(3), np.zeros(3)
        (r, p, y) = tf_transformations.euler_from_quaternion(quat)
        current_orientation = np.array([r, p, y])
        goal_orientation = np.array([0, 0, np.arctan2(linear_difference[1], linear_difference[0])])
        angular_err = self.calculate_angular_error(goal_orientation, current_orientation)
        if abs(angular_err[2]) < 0.3:
            self.sequence = PositionControllerSequence.GoToPoint
            self.goal_angle = goal_orientation
        return linear_err, angular_err
    
    def go_to_point(self, current_position, quat):
        # If close enough to point, ignore trajectory
        (r, p, y) = tf_transformations.euler_from_quaternion(quat)
        current_orientation = np.array([r, p, y])
        if np.linalg.norm(self.cmd_linear[:2] - current_position[:2]) < 0.2:
            self.sequence = PositionControllerSequence.MaintainPosition
            self.goal_pose = self.cmd_linear
            linear_err, angular_err = np.zeros(3), np.zeros(3)
        elif np.linalg.norm(self.cmd_linear[:2] - current_position[:2]) < 1.0:
            linear_err = self.calculate_linear_error(self.cmd_linear, current_position, quat)
            angular_err = self.calculate_angular_error(self.goal_angle, current_orientation)
        else:
            lookhead_distance = 1.0
            self.get_logger().info(f"{self.cmd_linear[:2]} {self.start_pose[:2]}")
            trajectory_vector = self.cmd_linear[:2] - self.start_pose[:2]
            trajectory_unit = trajectory_vector / np.linalg.norm(trajectory_vector)
            vec = current_position[:2] - self.start_pose[:2]
            proj_length = np.dot(vec, trajectory_unit)
            projected_point = self.start_pose[:2] + proj_length * trajectory_unit
            target_point = projected_point + lookhead_distance * trajectory_unit
            target_point = [target_point[0], target_point[1], self.cmd_linear[2]]
            linear_err = self.calculate_linear_error(target_point, current_position, quat)
            angular_err = self.calculate_angular_error(self.goal_angle, current_orientation)
        return linear_err, angular_err


def main(args=None):
    rclpy.init(args=args)
    node = LOSController()
    rclpy.spin(node)