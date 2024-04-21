import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool

from .PID import PID

class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.path_finished = True
        self.path = PoseArray()
        self.path_idx = 0

        # ROS infrastructure
        self.cmd_path_sub = self.create_subscription(
          PoseArray,
          '/cmd_path',
          self.cmd_path_callback,
          10)

        self.odom_sub = self.create_subscription(
          Bool,
          '/hold_point',
          self.position_callback,
          10)

        self.path_finished_pub = self.create_publisher(Bool, '/path_finished', 10)
        self.cmd_pos_pub = self.create_publisher(Pose, '/cmd_pos', 10)

    def cmd_path_callback(self, msg):
        self.path_finished = False
        self.path = msg
        
    def position_callback(self, msg):
        if self.path_finished:
            return

        if msg.data:
            self.path_idx += 1

        if self.path_idx >= len(self.path.poses):
            self.path_finished = True
            self.path_idx = 0
            self.path_finished_pub.publish(Bool(data=True))
            return

        pose = self.path.poses[self.path_idx]

        self.cmd_pos_pub.publish(pose)



def main(args=None):
    rclpy.init(args=args)

    path_follower_node = PathFollowerNode()
    rclpy.spin(path_follower_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
