import rclpy

from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

from pontus_autonomy.tasks.base_task import BaseTask

class ExitGate(BaseTask):
    def __init__(self):
        super().__init__("exit_gate")

        self.cmd_pose_pub = self.create_publisher(
            Pose,
            '/cmd_pos',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odom_callback,
            10,
        )    

        self.position_controller_state = self.create_subscription(
            Int32,
            '/pontus/position_controller_state_machine_status',
            self.position_controller_state_callback,
            10
        )

        self.current_pose = Pose()
        self.started = False
        self._done = False

        self.timer = self.create_timer(
            0.2, self.go
        )

        self.cmd_sent = False

    # Callbacks
    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose


    def position_controller_state_callback(self, msg: Int32):
        if not self.started and msg.data > 1:
            self.started = True
        if self.started and msg.data == 1:
            self._done = True
            self.started = False
    

    # Autonomy?
    def go(self):
        if not self.cmd_sent and self.current_pose is not None:
            cmd_pose = Pose()
            cmd_pose.position.z = self.current_pose.position.z
            cmd_pose.orientation.x = -1.0
            self.cmd_pose_pub.publish(cmd_pose)
            self.cmd_sent = True
        elif self.cmd_sent and self._done:
            self.complete(True)
        
        