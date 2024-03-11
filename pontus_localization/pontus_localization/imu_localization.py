import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class IMULocalizationNode(Node):

    def __init__(self):
        super().__init__('imu_localization')

        self.imu = None

        self.odom_pub = self.create_publisher(Odometry, '/pontus/odometry', 10)

        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.imu is None:
            self.odom_pub.publish(Odometry()) 
            return


def main(args=None):
    rclpy.init(args=args)

    node = IMULocalizationNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()