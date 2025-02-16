import rclpy
from rclpy.node import Node
import numpy as np
from enum import Enum

from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Bool
from rclpy.duration import Duration

from .PID import PID

class PositionNode(Node):
    
    class State(Enum):
      Maintain_position = 1
      Z_correction = 2
      Strafe = 3
      Direction_correction = 4
      Linear_correction = 5
      Angular_correction = 6

    def __init__(self):
        super().__init__('position_controller')
        self.state = self.State.Maintain_position
        self.transition_threshold = {}
        self.transition_threshold[self.State.Z_correction] = 0.07
        self.transition_threshold[self.State.Direction_correction] = 0.02
        self.transition_threshold[self.State.Linear_correction] = 0.1
        self.transition_threshold[self.State.Strafe] = 0.05
        self.transition_threshold[self.State.Angular_correction] = 0.05

        self.deadzone = 0.2
        self.stuck_error_threshold = 0.3
        self.stuck_error_time = Duration(seconds=7)

        self.cmd_linear = None
        self.cmd_angular = None

        self.prev_time = self.get_clock().now()
        self.prev_linear_time_under_threshold = self.get_clock().now()

        self.declare_parameter('sim_mode', False) 
        sim_mode = self.get_parameter('sim_mode').get_parameter_value().bool_value

        # TODO: Tune these
        # Sim PID values
        if sim_mode: 
            self.pid_linear = [
                PID(1, 0, 0.7), # X
                PID(1, 0, 0.5), # Y
                PID(2, 0, 2)  # Z
            ]
            self.pid_angular = [
                PID(0.5, 0, 0), # R
                PID(0.5, 0, 0), # P
                PID(1, 0, 0)  # Y
            ]
        # Real values for sub
        else:
            self.pid_linear = [
                PID(1.0, 0, 0), # X
                PID(0.5, 0, 0), # Y
                PID(5, 0, 0)  # Z
            ]
            
            self.pid_angular = [
                PID(0.5, 0, 0), # R
                PID(0.5, 0, 0), # P
                PID(0.5, 0, 0)  # Y
            ]

        self.thresh = 0.2
        self.angular_thresh = 0.05
        self.hold_point = False
        
        self.goal_pose = np.zeros(3)
        self.goal_angle = np.zeros(3)
        # ROS infrastructure
        self.cmd_pos_sub = self.create_subscription(
          Pose,
          '/cmd_pos',
          self.cmd_pos_callback,
          10)

        self.odom_sub = self.create_subscription(
          Odometry,
          '/pontus/odometry',
          self.odometry_callback,
          10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.hold_point_pub = self.create_publisher(Bool, '/hold_point', 10)

        self.previous_state = None

    def state_debugger(self):
        # self.get_logger().info(f"Goal pose: {self.goal_pose}")
        if self.previous_state != self.state:
            self.get_logger().info(f"Now at: {self.state.name}")
            self.previous_state = self.state


    def cmd_pos_callback(self, msg):
        # Store the commanded positions to use in the odometry callback
        quat = msg.orientation
        (r, p, y) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        cmd_pos_new = np.array([msg.position.x, msg.position.y, msg.position.z])
        cmd_ang_new = np.array([r, p, y])
        self.goal_pose = [self.goal_pose[0], self.goal_pose[1], cmd_pos_new[2]]
        # If the cmd_pos and cmd_ang is the same as the previous, do nothing
        if np.array_equal(cmd_pos_new, self.cmd_linear) and np.array_equal(cmd_ang_new, self.cmd_angular):
            return
        # If we get a new cmd_pos or cmd_ang, transition to the Z_correction state
        self.cmd_linear = cmd_pos_new
        self.cmd_angular = cmd_ang_new
        # self.state = self.State.Z_correction
        self.state = self.State.Strafe


    def angle_wrap(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi


    def odometry_callback(self, msg: Odometry):
        self.state_debugger()
        # # Get the current positions from odometry
        quat = msg.pose.pose.orientation
        quat = [quat.x, quat.y, quat.z, quat.w]

        # transform world frame to body frame
        u = np.array(quat[:3])
        u[1] = -u[1]
        u[2] = -u[2]
        s = quat[3]
        current_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        (r, p, y) = euler_from_quaternion(quat)
        
        quat = msg.pose.pose.orientation
        quat = [quat.x, quat.y, quat.z, quat.w]
        current_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        match self.state:
            case self.State.Maintain_position:
                linear_err, angular_err = self.maintain_position(current_position, quat)                
            # case self.State.Z_correction:
            #     linear_err, angular_err = self.correct_z(current_position, quat)
            case self.State.Strafe:
                linear_err, angular_err = self.correct_strafe(current_position, quat)
            case self.State.Direction_correction:
                linear_err, angular_err = self.turn_to_next_point(current_position, quat)
            case self.State.Linear_correction:
                linear_err, angular_err = self.go_to_point(current_position, quat)
            case self.State.Angular_correction:
                linear_err, angular_err = self.correct_orientation(current_position, quat)
            case _:
                linear_err = np.zeros(3)
                angular_err = np.zeros(3)

        # Compute and publish the body vel commands, we cannot move in y direction
        msg: Twist = Twist()
        msg.linear.x = self.pid_linear[0](linear_err[0], self.get_clock().now() - self.prev_time)
        msg.linear.y = self.pid_linear[1](linear_err[1], self.get_clock().now() - self.prev_time)
        msg.linear.z = self.pid_linear[2](linear_err[2], self.get_clock().now() - self.prev_time)

        msg.angular.x = self.pid_angular[0](angular_err[0], self.get_clock().now() - self.prev_time)
        msg.angular.y = self.pid_angular[1](angular_err[1], self.get_clock().now() - self.prev_time)
        msg.angular.z = self.pid_angular[2](angular_err[2], self.get_clock().now() - self.prev_time)

        self.cmd_vel_pub.publish(msg)
        self.hold_point_pub.publish(Bool(data=self.hold_point))

        self.prev_time = self.get_clock().now()

    
    def calculate_angular_error(self, desired_angle, current_angle):
        angular_diff = desired_angle - current_angle
        # find the shorter turn for angles
        angular_adj = np.sign(angular_diff) * 2 * np.pi
        angular_diff_alt = angular_diff - angular_adj
        angular_err = np.where(np.abs(angular_diff) < np.abs(angular_diff_alt), angular_diff, angular_diff_alt)
        return angular_err


    def calculate_linear_error(self, desired_pose, current_pose, quat_orientation):
        u = np.array(quat_orientation[:3])
        u[1] = -u[1]
        u[2] = -u[2]
        s = quat_orientation[3]
        linear_err = desired_pose - current_pose
        linear_err = 2.0 * np.dot(u, linear_err) * u + (s**2 - np.dot(u, u)) * linear_err + 2.0 * s * np.cross(u, linear_err)      
        return linear_err
    

    def maintain_position(self, current_position, quat):
        linear_err = self.calculate_linear_error(self.goal_pose, current_position, quat)
        (r, p, y) = euler_from_quaternion(quat)
        current_orientation = np.array([r, p, y])
        angular_err = self.calculate_angular_error(self.goal_angle, current_orientation)

        return linear_err, angular_err


    def correct_z(self, current_position, quat):
        self.goal_pose = [self.goal_pose[0], self.goal_pose[1], self.cmd_linear[2]]
        linear_err = self.calculate_linear_error(self.goal_pose, current_position, quat)
        (r, p, y) = euler_from_quaternion(quat)
        current_orientation = np.array([r, p, y])
        angular_err = self.calculate_angular_error(self.goal_angle, current_orientation)

        if abs(self.goal_pose[2] - current_position[2]) < self.transition_threshold[self.State.Z_correction]:
            self.state = self.State.Direction_correction
        return linear_err, angular_err
    

    def correct_strafe(self, current_position, quat):
        linear_err = self.calculate_linear_error(self.cmd_linear, current_position, quat)
        # If distance super far, do not strafe, more efficient to turn and go forward
        if np.linalg.norm(linear_err[:2]) > 1.0:
            self.state = self.State.Direction_correction
            return np.zeros(3), np.zeros(3)
        
        self.goal_pose = self.cmd_linear
        # If distance is not super far, strafe
        (r, p, y) = euler_from_quaternion(quat)
        current_orientation = np.array([r, p, y])
        angular_err = self.calculate_angular_error(self.goal_angle, current_orientation)

        if np.linalg.norm(linear_err[:2]) < self.transition_threshold[self.State.Strafe]:
            self.state = self.State.Direction_correction
        return linear_err, angular_err

    def turn_to_next_point(self, current_position, quat):
        linear_err = self.calculate_linear_error(self.goal_pose, current_position, quat)

        linear_difference = self.cmd_linear - current_position
        if np.linalg.norm(linear_difference[:2]) < self.deadzone:
            self.state = self.State.Angular_correction
        
        (r, p, y) = euler_from_quaternion(quat)
        current_orientation = np.array([r, p, y])
        goal_orientation = np.array([0, 0, np.arctan2(linear_difference[1], linear_difference[0])])
        angular_err = self.calculate_angular_error(goal_orientation, current_orientation)
        
        if np.linalg.norm(angular_err) < self.transition_threshold[self.State.Direction_correction]:
            self.state = self.State.Linear_correction
            self.goal_angle = goal_orientation
            self.prev_linear_time_under_threshold = self.get_clock().now()
        return linear_err, angular_err
    

    def go_to_point(self, current_position, quat):
        # self.get_logger().info(f"{self.prev_linear_time_under_threshold}")
        self.goal_pose = self.cmd_linear
        linear_err = self.calculate_linear_error(self.goal_pose, current_position, quat)
        angular_err = np.zeros(3)

        if np.linalg.norm(linear_err) < self.transition_threshold[self.State.Linear_correction]:
            # self.state = self.State.Angular_correction
            self.state = self.State.Angular_correction

            (r, p, y) = euler_from_quaternion(quat)
            current_orientation = np.array([r, p, y])
            self.goal_angle = current_orientation

        # If we ever get stuck here, transition back to turn_to_point
        if np.linalg.norm(linear_err) <= self.stuck_error_threshold and self.get_clock().now() - self.prev_linear_time_under_threshold > self.stuck_error_time:
            self.state = self.State.Direction_correction
            self.goal_pose = self.cmd_linear
        elif np.linalg.norm(linear_err) > self.stuck_error_threshold:
            self.prev_linear_time_under_threshold = self.get_clock().now()
        return linear_err, angular_err


    def correct_orientation(self, current_position, quat):
        linear_err = self.calculate_linear_error(self.goal_pose, current_position, quat)

        (r, p, y) = euler_from_quaternion(quat)
        current_orientation = np.array([r, p, y])
        angular_err = self.calculate_angular_error(self.cmd_angular, current_orientation)

        if np.linalg.norm(angular_err) < self.transition_threshold[self.State.Angular_correction]:
          self.goal_angle = self.cmd_angular
          self.state = self.State.Maintain_position
        return linear_err, angular_err


def main(args=None):
    rclpy.init(args=args)

    position_node = PositionNode()
    rclpy.spin(position_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()