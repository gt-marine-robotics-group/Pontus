import rclpy
from rclpy.node import Node
import numpy as np

import tf_transformations
from typing import Optional, List
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from pontus_controller.PID import PID, DegreeOfFreedom


class HybridControllerNode(Node):
    def __init__(self):
        super().__init__('hybrid_controller')

        self.cmd_linear = np.zeros(3)
        self.cmd_angular = np.zeros(3)

        self.prev_time = self.get_clock().now()

        self.pid_linear = [
          PID(2, 0, 0, degree_of_freedom=DegreeOfFreedom.SURGE),
          PID(2, 0.0, 0.0, degree_of_freedom=DegreeOfFreedom.SWAY, windup_max= 10),
          PID(1, 0.0, 0.0, degree_of_freedom=DegreeOfFreedom.HEAVE)
        ]
        self.pid_angular = [
          PID(1.0, 0.0, 0.0, degree_of_freedom=DegreeOfFreedom.ROLL),
          PID(0.01, 0.0, 0.0, degree_of_freedom=DegreeOfFreedom.PITCH),
          PID(0.01, 0, 0.0, degree_of_freedom=DegreeOfFreedom.YAW, windup_max=1)
        ]

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

        # Compute and publish the body accelerations
        accel_msg = Twist()

        # Get the orientation from odometry
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        roll, pitch, yaw = tf_transformations.euler_from_quaternion((x, y, z, w))

        # Enable active roll/pitch control to 0
        # accel_msg.angular.x = self.pid_angular[0](-roll,
        #                                           self.get_clock().now() - self.prev_time)
        # accel_msg.angular.y = self.pid_angular[1](-pitch, self.get_clock().now() - self.prev_time)
        # self.get_logger().info(str(accel_msg.angular.y))

        # Direct command yaw
        accel_msg.angular.z = self.cmd_angular[2]

        # Direct command linear x, y
        accel_msg.linear.x = 5 * self.cmd_linear[0]
        accel_msg.linear.y = 5 * self.cmd_linear[1]

        dt = self.get_clock().now() - self.prev_time
        z_err = self.cmd_depth - self.depth_msg.pose.pose.position.z
        accel_msg.linear.z = self.pid_linear[2](z_err, dt)

        # Maintain depth if no commanded velocity
        if abs(self.cmd_linear[2]) < 0.05:
            z_err = self.cmd_depth - self.depth_msg.pose.pose.position.z
            accel_msg.linear.z = self.pid_linear[2](z_err, dt)

        # Follow commanded z velocity
        else:
            self.cmd_depth = self.depth_msg.pose.pose.position.z
            accel_msg.linear.z = 16.0 * self.cmd_linear[2]

        self.cmd_accel_pub.publish(accel_msg)

        self.prev_time = self.get_clock().now()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)

    hybrid_node = HybridControllerNode()
    rclpy.spin(hybrid_node)

    rclpy.shutdown()
