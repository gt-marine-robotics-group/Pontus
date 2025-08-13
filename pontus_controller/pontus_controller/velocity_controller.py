import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from typing import Optional, List
from pontus_controller.PID import PID, FeedForwardPID, DegreeOfFreedom


class VelocityNode(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        self.cmd_linear = np.zeros(3)
        self.cmd_angular = np.zeros(3)

        self.prev_time = self.get_clock().now()

        param_list = (
            ('x_kp', 10.0),
            ('x_ki', 0.0),
            ('x_kd', 0.0),
            ('y_kp', 10.0),
            ('y_ki', 1.0),
            ('y_kd', 0.000001),
            ('z_kp', 10.0),
            ('z_ki', 0.0),
            ('z_kd', 0.0),
            ('r_kp', 1.0),
            ('r_ki', 0.0),
            ('r_kd', 0.0),
            ('p_kp', 0.01),
            ('p_ki', 0.0),
            ('p_kd', 0.0),
            ('yaw_kp', 0.1),
            ('yaw_ki', 0.0),
            ('yaw_kd', 0.0001),
        )

        self.pids_created = False
        self.add_on_set_parameters_callback(self.param_callback)
        self.declare_parameters(namespace="vel", parameters=param_list)

        # TODO: Tune these
        self.pid_linear = [
          FeedForwardPID(self.x_kp, self.x_ki, self.x_kd, degree_of_freedom=DegreeOfFreedom.SURGE),  # X
          FeedForwardPID(self.y_kp, self.y_ki, self.y_kd, degree_of_freedom=DegreeOfFreedom.SWAY, windup_max=10),  # Y
          FeedForwardPID(self.z_kp, self.z_ki, self.z_kd, degree_of_freedom=DegreeOfFreedom.HEAVE)  # Z
        ]

        self.pid_angular = [
          FeedForwardPID(self.r_kp, self.r_ki, self.r_kd, degree_of_freedom=DegreeOfFreedom.ROLL),  # R
          FeedForwardPID(self.p_kp, self.p_ki, self.p_kd, degree_of_freedom=DegreeOfFreedom.PITCH),  # P
          FeedForwardPID(self.yaw_kp, self.yaw_ki, self.yaw_kd, degree_of_freedom=DegreeOfFreedom.YAW, windup_max=1)  # Y
        ]
        self.pids_created = True

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
        msg.angular.y = self.pid_angular[1](angular_err[1], dt, self.cmd_angular[1])
        msg.angular.z = self.pid_angular[2](angular_err[2], dt, self.cmd_angular[2])
        self.prev_time = self.get_clock().now()

        self.cmd_accel_pub.publish(msg)

    def param_callback(self, params):
      for param in params:
        name = param.name.replace("vel.", "")
        setattr(self, name, param.value)

        if self.pids_created:
          split = name.split("_")
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

          if pid_obj is not None:
            if isinstance(pid_obj, FeedForwardPID):
              setattr(pid_obj.pid, gain, param.value)
            elif isinstance(pid_obj, PID):
              setattr(pid_obj, gain, param.value)

      return SetParametersResult(successful=True)

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    velocity_Node = VelocityNode()
    rclpy.spin(velocity_Node)
    rclpy.shutdown()
