import rclpy
from rclpy.node import Node
import numpy as np

import tf_transformations
from typing import Optional, List
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult

from pontus_controller.PID import PID


class HybridControllerNode(Node):
    def __init__(self):
        super().__init__('hybrid_controller')

        self.cmd_linear = np.zeros(3)
        self.cmd_angular = np.zeros(3)

        self.prev_time = self.get_clock().now()

        param_list = (
            ('x_kp', 2.0),
            ('x_ki', 0.0),
            ('x_kd', 0.0),
            ('y_kp', 2.0),
            ('y_ki', 0.0),
            ('y_kd', 0.0),
            ('z_kp', 3.0),
            ('z_ki', 0.0),
            ('z_kd', 0.1),
            ('r_kp', 1.0),
            ('r_ki', 0.0),
            ('r_kd', 0.0),
            ('p_kp', 0.01),
            ('p_ki', 0.0),
            ('p_kd', 0.0),
            ('yaw_kp', 4.0),
            ('yaw_ki', 0.0),
            ('yaw_kd', 4.0),
        )

        self.pids_created = False
        self.add_on_set_parameters_callback(self.param_callback)
        self.declare_parameters(namespace="hybrid", parameters=param_list)

        self.pid_linear = [
          PID(self.x_kp, self.x_ki, self.x_kd),
          PID(self.y_kp, self.y_ki, self.y_kd, windup_max= 10),
          PID(self.z_kp, self.z_ki, self.z_kd, windup_max = 1.0)
        ]
        self.pid_angular = [
          PID(self.r_kp, self.r_ki, self.r_kd),
          PID(self.p_kp, self.p_ki, self.p_kd),
          PID(self.yaw_kp, self.yaw_ki, self.yaw_kd, windup_max=1.0)
        ]

        self.pids_created = True

        # ROS infrastructure
        self.cmd_vel_sub = self.create_subscription(
          Twist,
          '/cmd_vel',
          self.cmd_vel_callback,
          10)

        # Nonfiltered depth only
        self.depth_sub = self.create_subscription(
          Odometry,
          '/pontus/depth_0',
          self.depth_callback,
          10)

        # Full odometry
        self.odom_sub = self.create_subscription(
          Odometry,
          '/pontus/odometry',
          self.odometry_callback,
          10)

        self.cmd_accel_pub = self.create_publisher(Twist, '/cmd_accel', 10)

        # self.cmd_depth = None
        self.cmd_depth = -0.5
        self.depth_msg = Odometry()

    def depth_callback(self, msg: Odometry) -> None:
        """
        Save depth from depth sensor.

        Args:
        ----
        msg (Odometry): depth message

        Return:
        ------
        None

        """
        self.depth_msg = msg

    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Store the commanded velocities to use in the odometry callback.

        Args:
        ----
        msg (Twist): command velocity from topic

        Return:
        ------
        None

        """
        self.cmd_linear = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
        self.cmd_angular = np.array([msg.angular.x, msg.angular.y, msg.angular.z])

    def odometry_callback(self, msg: Odometry) -> None:
        """
        Convert command velocity to acceleration.

        This says odometry, but we are really just using the IMU data from robot localization.
        Since there is no proper linear feedback, we will just guess the acceleration we need to
        go to achieve a certain velocity.

        Args:
        ----
        msg (Odometry): odometry message from topic

        Return:
        ------
        None

        """
        self.depth_msg = msg

        self.get_logger().info(f"pid yaw kp: {self.pid_angular[2].kp}")
        self.get_logger().info(f"cmd_depth: {self.cmd_depth}")

        # Compute and publish the body accelerations
        accel_msg = Twist()

        # Get the orientation from odometry
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        roll, pitch, yaw = tf_transformations.euler_from_quaternion((x, y, z, w))

        dt = self.get_clock().now() - self.prev_time

        # PID feedback controlled xy + yaw
        accel_msg.linear.x = self.pid_linear[0](self.cmd_linear[0] - msg.twist.twist.linear.x, dt)
        accel_msg.linear.y = self.pid_linear[1](self.cmd_linear[1] - msg.twist.twist.linear.y, dt)

        accel_msg.angular.z = self.pid_angular[2](self.cmd_angular[2] - msg.twist.twist.angular.z, dt)

        # Enable active roll/pitch control to 0
        # accel_msg.angular.x = self.pid_angular[0](-roll,
        #                                           self.get_clock().now() - self.prev_time)
        # accel_msg.angular.y = self.pid_angular[1](-pitch, self.get_clock().now() - self.prev_time)
        # self.get_logger().info(str(accel_msg.angular.y))

        # Direct command yaw
        # accel_msg.angular.z = self.cmd_angular[2]

        # Direct command linear x, y
        # accel_msg.linear.x = 5 * self.cmd_linear[0]
        # accel_msg.linear.y = 5 * self.cmd_linear[1]

        # z_err = self.cmd_depth - self.depth_msg.pose.pose.position.z
        # accel_msg.linear.z = self.pid_linear[2](z_err, dt)

        # Maintain depth if no commanded velocity
        if abs(self.cmd_linear[2]) < 0.05:
            z_err = self.cmd_depth - self.depth_msg.pose.pose.position.z
            accel_msg.linear.z = self.pid_linear[2](z_err, dt)

        # Follow commanded z velocity
        else:
            self.cmd_depth = self.depth_msg.pose.pose.position.z
            accel_msg.linear.z = self.pid_linear[2](self.cmd_linear[2] - msg.twist.twist.linear.z, dt)
            # accel_msg.linear.z = 16.0 * self.cmd_linear[2]

        accel_msg.linear.z -= 9.8 * ((1000 * 0.0405) - (34.02)) / 34.02 # TEMPORARY FEEDFORWARD
        self.cmd_accel_pub.publish(accel_msg)

        self.prev_time = self.get_clock().now()

    def param_callback(self, params):
      for param in params:
        name = param.name.replace("hybrid.", "")
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

          if (pid_obj is not None):
            setattr(pid_obj, gain, param.value)

      return SetParametersResult(successful=True)

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)

    hybrid_node = HybridControllerNode()
    rclpy.spin(hybrid_node)

    rclpy.shutdown()
