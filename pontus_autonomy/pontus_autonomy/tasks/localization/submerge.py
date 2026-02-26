from geometry_msgs.msg import Pose
from pontus_msgs.msg import CommandMode

from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj
from pontus_autonomy.tasks.base_task import BaseTask


class Submerge(BaseTask):
    def __init__(self):
        super().__init__('submerge')

        # Hyperparameters

        # Determines the desired depth the sub should start autonomy
        # TODO: See if the DVL is able to get this
        self.desired_depth = -1.5

        # End

        self.go_to_pose_client = GoToPoseClient(self)

        self.timer = self.create_timer(
            0.2, self.submerge
        )
        self.cmd_sent = False

    def submerge(self) -> None:
        """
        Command sub to submerge.

        Before completing any tasks, the sub must first submerge. This task will send the sub to
        a desired depth.

        Args:
        ----
        None

        Return:
        ------
        None

        """
        if not self.cmd_sent:
            lower_pose = Pose()
            lower_pose.position.z = self.desired_depth
            pose_obj = PoseObj(cmd_pose=lower_pose)
            self.go_to_pose_client.go_to_pose(pose_obj)
            self.cmd_sent = True
        if self.go_to_pose_client.at_pose():
            self.complete(True)
