from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj
from pontus_autonomy.tasks.base_task import BaseTask


class PrequalReturn(BaseTask):
    def __init__(self):
        super().__init__("vertical_to_gate")

        self.go_to_pose_client = GoToPoseClient(self)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odometry_callback,
            10
        )

        self.current_pose = None
        self.timer = self.create_timer(
            0.2, self.go
        )
        self.cmd_sent = False

    def odometry_callback(self, msg: Odometry) -> None:
        """
        Handle odometry callback.

        Args:
        ----
        msg (Odometry): the odometry message from the topic

        Return:
        ------
        None

        """
        self.current_pose = msg.pose.pose

    # Autonomy
    def go(self) -> None:
        """
        Command AUV to return to the gate.

        Args:
        ----
        None

        Return:
        ------
        None

        """
        if not self.cmd_sent and self.current_pose:
            cmd_pose = Pose()
            cmd_pose.position.z = self.current_pose.position.z
            self.go_to_pose_client.go_to_pose(PoseObj(cmd_pose=cmd_pose, skip_orientation=True))
            self.cmd_sent = True

        elif self.cmd_sent and self.go_to_pose_client.at_pose():
            self.complete(True)
