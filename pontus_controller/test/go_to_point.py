import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler

import numpy as np

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.vel_pub = self.create_publisher(Pose, '/cmd_pos', 10)

        self.msg = Pose()

        self.msg.position.x = 0.0
        self.msg.position.y = 0.0
        self.msg.position.z = 0.0

        x, y, z, w = quaternion_from_euler(0, 0, np.pi/2)

        self.msg.orientation.x = x
        self.msg.orientation.y = y
        self.msg.orientation.z = z
        self.msg.orientation.w = w


        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.vel_pub.publish(self.msg)

        self.get_logger().info('Publishing: "%s"' % self.msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
