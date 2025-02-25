from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient

class ExitGate(BaseTask):
    def __init__(self):
        super().__init__("exit_gate")

        self.odom_sub = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odom_callback,
            10,
        )    
        self.go_to_pose_client = GoToPoseClient(self)
        self.current_pose = Pose()

        self.timer = self.create_timer(
            0.2, self.go
        )

        self.cmd_sent = False

    # Callbacks
    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
    
    # Autonomy?
    def go(self):
        if not self.cmd_sent and self.current_pose is not None:
            cmd_pose = Pose()
            cmd_pose.position.z = self.current_pose.position.z
            self.go_to_pose_client.go_to_pose(cmd_pose)
            self.cmd_sent = True
        elif self.cmd_sent and self.go_to_pose_client.at_pose():
            self.complete(True)
        
        