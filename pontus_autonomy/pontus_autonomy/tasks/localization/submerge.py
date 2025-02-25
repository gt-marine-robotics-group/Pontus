from geometry_msgs.msg import Pose

from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient
from pontus_autonomy.tasks.base_task import BaseTask

class Submerge(BaseTask):
    def __init__(self):
        super().__init__('submerge')

        self.go_to_pose_client = GoToPoseClient(self)

        self.desired_depth = -1.5


        self.timer = self.create_timer(
            0.2, self.submerge
        )
        self.cmd_sent = False


    def submerge(self):
        if not self.cmd_sent:
            lower_pose = Pose()
            lower_pose.position.z = self.desired_depth
            self.go_to_pose_client.go_to_pose(lower_pose)
            self.cmd_sent = True
        if self.go_to_pose_client.at_pose():
            self.complete(True)