import rclpy
from rclpy.node import Node
import numpy as np

import tf2_ros
import tf_transformations
# sudo pip3 install transforms3d
# sudo apt install ros-humble-tf-transformations

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from .PID import PID

class HybridControllerNode(Node):

    def __init__(self):
        super().__init__('hybrid_controller')

        self.cmd_linear = np.zeros(3)
        self.cmd_angular = np.zeros(3)

        self.prev_time = self.get_clock().now()

        # TODO: Tune these
        self.pid_linear = [
          PID(2, 0, 0, 2), # X
          PID(2, 0, 0, 2), # Y
          PID(32, 0.0, 35.0, 0.0)  # Z
        ]

        self.pid_angular = [
          PID(0.5, 0, 0, 2), # R
          PID(40, 0.0, 6, 1), # P
          PID(2, 0, 0, 2)  # Y
        ]

        # ROS infrastructure
        self.cmd_vel_sub = self.create_subscription(
          Twist,
          '/cmd_vel',
          self.cmd_vel_callback,
          10)

        # Nonfiltered depth only
        self.odom_sub = self.create_subscription(
          Odometry,
          '/pontus/depth_0',
          self.odometry_callback,
          10)

        # Full odometry
        self.odom_sub = self.create_subscription(
          Odometry,
          '/pontus/odometry',
          self.odometry_callback,
          10)

        self.cmd_accel_pub = self.create_publisher(Twist, '/cmd_accel', 10)

        #self.cmd_depth = None
        self.cmd_depth = -1.2


    def cmd_vel_callback(self, msg):
        # Store the commanded velocities to use in the odometry callback
        self.cmd_linear = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
        self.cmd_angular = np.array([msg.angular.x, msg.angular.y, msg.angular.z])

    def odometry_callback(self, msg):


        # Compute and publish the body accelerations
        accel_msg = Twist()

        # Get the orientation from odometry
        x, y, z, w = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        roll, pitch, yaw = tf_transformations.euler_from_quaternion((x, y, z, w))

        # Enable active roll/pitch control to 0
        #accel_msg.angular.x = self.pid_angular[0](-roll, self.get_clock().now() - self.prev_time)
        accel_msg.angular.y = self.pid_angular[1](-pitch, self.get_clock().now() - self.prev_time)
        # self.get_logger().info(str(accel_msg.angular.y))

        # Direct command yaw
        accel_msg.angular.z = self.cmd_angular[2]

        # Direct command linear x, y
        accel_msg.linear.x = 0.7 * self.cmd_linear[0]
        # accel_msg.linear.y = 0.2 * self.cmd_linear[1]

        #accel_msg.linear.z = self.pid_linear[2](self.cmd_depth - msg.pose.pose.position.z, self.get_clock().now() - self.prev_time)

        # Maintain depth if no commanded velocity
        if abs(self.cmd_linear[2]) < 0.05:
            accel_msg.linear.z = self.pid_linear[2](self.cmd_depth - msg.pose.pose.position.z, self.get_clock().now() - self.prev_time)

        # Follow commanded z velocity
        else:
            self.cmd_depth = msg.pose.pose.position.z
            accel_msg.linear.z = 16.0 * self.cmd_linear[2]

        self.cmd_accel_pub.publish(accel_msg)

        self.prev_time = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)

    hybrid_node = HybridControllerNode()
    rclpy.spin(hybrid_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
