import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

from .PID import PID

class PositionNode(Node):
    def __init__(self):
        super().__init__('position_controller')

        self.cmd_linear = np.zeros(3)
        self.cmd_angular = np.zeros(3)

        self.prev_time = self.get_clock().now()

        # TODO: Tune these
        self.pid_linear = [
          PID(0.5, 0, 0), # X
          PID(0.5, 0, 0), # Y
          PID(0.5, 0, 0)  # Z
        ]

        self.pid_angular = [
          PID(0.5, 0, 0), # R
          PID(0.5, 0, 0), # P
          PID(0.5, 0, 0)  # Y
        ]

        # ROS infrastructure
        self.cmd_pos_sub = self.create_subscription(
          Twist,
          '/cmd_pos',
          self.cmd_pos_callback,
          10)

        self.odom_sub = self.create_subscription(
          Odometry,
          '/pontus/odometry',
          self.odometry_callback,
          10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)


    def cmd_pos_callback(self, msg):
        # Store the commanded positions to use in the odometry callback
        self.cmd_linear = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
        self.cmd_angular = np.array([msg.angular.x, msg.angular.y, msg.angular.z])


    def odometry_callback(self, msg):
        # Get the current positions from odometry
        pose = msg.pose.pose.point
        quat = msg.pose.pose.orientation
        (r, p, y) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])


        p_linear = np.array([pose.x, pose.y, pose.z])
        p_angular = np.array([r, p ,y])

        # Compute the error between desired position and current position
        linear_err = self.cmd_linear - p_linear
        angular_diff = self.cmd_angular - p_angular
        
        # find the shorter turn for angles
        angular_adj = np.sign(angular_diff) * 2 * np.pi
        angular_diff_alt = angular_diff - angular_adj
        angular_err = np.where(np.abs(angular_diff) < np.abs(angular_diff_alt), angular_diff, angular_diff_alt)

        # Compute and publish the body vel commands
        msg = Twist()
        msg.linear.x = self.pid_linear[0](linear_err[0], self.get_clock().now() - self.prev_time)
        msg.linear.y = self.pid_linear[1](linear_err[1], self.get_clock().now() - self.prev_time)
        msg.linear.z = self.pid_linear[2](linear_err[2], self.get_clock().now() - self.prev_time)

        msg.angular.x = self.pid_angular[0](angular_err[0], self.get_clock().now() - self.prev_time)
        msg.angular.y = self.pid_angular[1](angular_err[1], self.get_clock().now() - self.prev_time)
        msg.angular.z = self.pid_angular[2](angular_err[2], self.get_clock().now() - self.prev_time)

        self.cmd_vel_pub.publish(msg)

        self.prev_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)

    position_node = PositionNode()
    rclpy.spin(position_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
