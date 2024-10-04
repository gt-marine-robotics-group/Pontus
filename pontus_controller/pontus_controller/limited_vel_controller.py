import rclpy
from rclpy.node import Node
import numpy as np

import tf2_ros

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from .PID import PID

class LimitedVelNode(Node):

    def __init__(self):
        super().__init__('limited_vel_controller')

        self.cmd_linear = np.zeros(3)
        self.cmd_angular = np.zeros(3)

        self.prev_time = self.get_clock().now()

        # TODO: Tune these
        self.pid_linear = [
          PID(2, 0, 0, 2), # X
          PID(2, 0, 0, 2), # Y
          PID(40, 0.0, 10.0, 0.5)  # Z
        ]

        self.pid_angular = [
          PID(0.005, 0, 15.0, 2), # R
          PID(0.005, 0, 15.0, 2), # P
          PID(0.5, 0, 0, 2)  # Y
        ]

        # ROS infrastructure
        self.cmd_vel_sub = self.create_subscription(
          Twist,
          '/cmd_vel',
          self.cmd_vel_callback,
          10)

        self.odom_sub = self.create_subscription(
          Odometry,
          '/dvl/odometry',
          self.odometry_callback,
          10)

        self.cmd_accel_pub = self.create_publisher(Twist, '/cmd_accel', 10)


    def cmd_vel_callback(self, msg):
        # Store the commanded velocities to use in the odometry callback
        self.cmd_linear = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
        self.cmd_angular = np.array([msg.angular.x, msg.angular.y, msg.angular.z])

    def odometry_callback(self, msg):

        # linear = np.array((msg.linear.x, msg.linear.y, msg.linear.z)) * 12.0
        # angular = np.array((msg.angular.x, msg.angular.y, msg.angular.z)) / 3.0

        msg.twist.twist.linear.z = -msg.twist.twist.linear.z
        # Get the current velocities from odometry
        v_linear = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        v_angular = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])

        # Compute the error between desired velocity and current velocity
        linear_err = self.cmd_linear - v_linear
        angular_err = self.cmd_angular - v_angular

        # Compute and publish the body accelerations
        msg = Twist()
        msg.linear.x = 12.0 * self.cmd_linear[0]
        msg.linear.y = 12.0 * self.cmd_linear[1]
        msg.linear.z = self.pid_linear[2](linear_err[2], self.get_clock().now() - self.prev_time)

        #msg.angular.x = self.pid_angular[0](angular_err[0], self.get_clock().now() - self.prev_time)
        #msg.angular.y = self.pid_angular[1](angular_err[1], self.get_clock().now() - self.prev_time)
        msg.angular.x = self.cmd_angular[0] / 3.0
        msg.angular.y = self.cmd_angular[1] / 3.0

        msg.angular.z = self.cmd_angular[2] / 3.0

        self.cmd_accel_pub.publish(msg)

        self.prev_time = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)

    limited_vel_Node = LimitedVelNode()
    rclpy.spin(limited_vel_Node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
