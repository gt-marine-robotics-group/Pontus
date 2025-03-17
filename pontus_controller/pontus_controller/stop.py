import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from typing import Optional, List


class Stop(Node):
    def __init__(self):
        super().__init__('stop')

        self.pub = self.create_publisher(Float32MultiArray, 'thrust_cmds', 10)

        self.create_timer(0.01, self.callback)

    def callback(self) -> None:
        """
        Command AUV to stop.

        Args:
        ----
            None

        Return:
        ------
            None

        """
        msg = Float32MultiArray()
        msg.data = [0.0 for i in range(8)]
        self.pub.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = Stop()
    rclpy.spin(node)
