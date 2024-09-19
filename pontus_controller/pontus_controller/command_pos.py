import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose

class CommandPoseNode(Node):
    def __init__(self):
        super().__init__('command_pose')

        self.cmd_accel_pub = self.create_publisher(Pose, '/cmd_pos', 10)

def main(args=None):
    rclpy.init(args=args)

    command_pose_node = CommandPoseNode()
    rclpy.spin(command_pose_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
