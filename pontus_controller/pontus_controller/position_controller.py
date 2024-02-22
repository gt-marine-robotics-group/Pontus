import rclpy
from rclpy.node import Node
import numpy as np

<<<<<<< HEAD
from geometry_msgs.msg import Pose, Twist
=======
from geometry_msgs.msg import Twist
>>>>>>> 65df69e (add waypoint controller)
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

<<<<<<< HEAD
        self.thresh = 0.1

        # ROS infrastructure
        self.cmd_pos_sub = self.create_subscription(
          Pose,
=======
        # ROS infrastructure
        self.cmd_pos_sub = self.create_subscription(
          Twist,
>>>>>>> 65df69e (add waypoint controller)
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
        quat = msg.orientation
        (r, p, y) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.cmd_linear = [msg.position.x, msg.position.y, msg.position.z]
        self.cmd_angular = np.array([r, p, y])


    def odometry_callback(self, msg):
        # Get the current positions from odometry
        quat = msg.pose.pose.orientation
        quat = [quat.x, quat.y, quat.z, quat.w]

        # transform world frame to body frame
        u = np.array(quat[:3])
        s = quat[3]
        linear_err = self.cmd_linear - np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        linear_err = 2.0 * np.dot(u, linear_err) * u + (s**2 - np.dot(u, u)) * linear_err + 2.0 * s * np.cross(u, linear_err)
        
        # find angular error
        (r, p, y) = euler_from_quaternion(quat)
        p_angular = np.array([r, p ,y])
        angular_diff = (np.array([0, np.arctan2(self.cmd_linear[2], self.cmd_linear[0]), np.arctan2(self.cmd_linear[1], self.cmd_linear[0])]) 
                        if np.linalg.norm(linear_err) > self.thresh else self.cmd_angular) - p_angular
        
        # find the shorter turn for angles
        angular_adj = np.sign(angular_diff) * 2 * np.pi
        angular_diff_alt = angular_diff - angular_adj
        angular_err = np.where(np.abs(angular_diff) < np.abs(angular_diff_alt), angular_diff, angular_diff_alt)

        # Compute and publish the body vel commands, we cannot move in y direction
        msg = Twist()
        msg.linear.x = self.pid_linear[0](linear_err[0], self.get_clock().now() - self.prev_time)
        msg.linear.y = 0.0
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
