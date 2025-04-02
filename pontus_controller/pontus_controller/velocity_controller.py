import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from typing import Optional, List
from pontus_controller.PID import PID, DegreeOfFreedom


class VelocityNode(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        self.cmd_linear = np.zeros(3)
        self.cmd_angular = np.zeros(3)

        self.prev_time = self.get_clock().now()

        # TODO: Tune these
        self.pid_linear = [
          PID(20, 0, 0, degree_of_freedom=DegreeOfFreedom.SURGE),  # X
          PID(20, 0, 0, degree_of_freedom=DegreeOfFreedom.SWAY),  # Y
          PID(10, 0, 0, degree_of_freedom=DegreeOfFreedom.HEAVE)  # Z
        ]

        self.pid_angular = [
          PID(1, 0, 0, degree_of_freedom=DegreeOfFreedom.ROLL),  # R
          PID(1, 0, 0, degree_of_freedom=DegreeOfFreedom.PITCH),  # P
          PID(0.01, 0, 0.0000001, degree_of_freedom=DegreeOfFreedom.YAW)  # Y
        ]

        # Sim PID values
        # self.pid_linear = [
        #   PID(2, 0.2, 0, 2), # X
        #   PID(2, 1, 0, 2), # Y
        #   PID(2, 1, 0, 2)  # Z
        # ]

        # ROS infrastructure
        self.cmd_vel_sub = self.create_subscription(
          Twist,
          '/cmd_vel',
          self.cmd_vel_callback,
          10)

        self.odom_sub = self.create_subscription(
          Odometry,
          '/pontus/odometry',
          self.odometry_callback,
          10)

        self.cmd_accel_pub = self.create_publisher(Twist, '/cmd_accel', 10)

    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Save command velocity to then use in odometry callback.

        Args:
        ----
        msg (Twist): the desired twist

        Return:
        ------
        None

        """
        self.cmd_linear = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
        self.cmd_angular = np.array([msg.angular.x, msg.angular.y, msg.angular.z])

    def odometry_callback(self, msg: Odometry) -> None:
        """
        Use odometry as feedback to command a desired acceleration to reach a desired velocity.

        Args:
        ----
        msg (Odometry): our current odometry

        Return:
        ------
        None

        """
        # Get the current velocities from odometry
        v_linear = np.array([msg.twist.twist.linear.x,
                             msg.twist.twist.linear.y,
                             msg.twist.twist.linear.z])
        v_angular = np.array([msg.twist.twist.angular.x,
                              msg.twist.twist.angular.y,
                              msg.twist.twist.angular.z])

        # Compute the error between desired velocity and current velocity
        linear_err = self.cmd_linear - v_linear
        angular_err = self.cmd_angular - v_angular

        # Compute and publish the body accelerations
        msg = Twist()
        dt = self.get_clock().now() - self.prev_time
        msg.linear.x = self.pid_linear[0](linear_err[0], dt)
        msg.linear.y = self.pid_linear[1](linear_err[1], dt)
        msg.linear.z = self.pid_linear[2](linear_err[2], dt)
        # msg.angular.x = self.pid_angular[0](angular_err[0], dt, self.cmd_angular[0])
        # msg.angular.y = self.pid_angular[1](angular_err[1], dt, self.cmd_angular[1])
        msg.angular.z = self.pid_angular[2](angular_err[2], dt, self.cmd_angular[2])
        self.prev_time = self.get_clock().now()

        self.cmd_accel_pub.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    velocity_Node = VelocityNode()
    rclpy.spin(velocity_Node)
    rclpy.shutdown()
