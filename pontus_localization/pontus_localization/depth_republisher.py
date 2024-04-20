import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from .pf_constants import *

class DepthRepublisher(Node):
    def __init__(self):
        super().__init__("depth_republisher_node")

        self.create_subscription(
            Odometry,
            "/pontus/odometry",
            self.odom_callback,
            10,
        )

        self.depth_pub = self.create_publisher(
            Odometry,
            "/pontus/depth_0",
            10,
        )

    def odom_callback(self, msg):
        depth = msg.pose.pose.position.z
        pub_msg = Odometry()
        pub_msg.pose.pose.position.z = depth
        self.depth_pub.publish(pub_msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = DepthRepublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()