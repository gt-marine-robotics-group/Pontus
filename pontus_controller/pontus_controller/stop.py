import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class Stop(Node):

    def __init__(self):
        super().__init__('stop')

        self.pub = self.create_publisher(Float32MultiArray, 'thrust_cmds', 10)

        self.create_timer(0.01, self.callback)

    def callback(self):

        msg = Float32MultiArray()
        msg.data = [0.0 for i in range(8)]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Stop()
    rclpy.spin(node)
