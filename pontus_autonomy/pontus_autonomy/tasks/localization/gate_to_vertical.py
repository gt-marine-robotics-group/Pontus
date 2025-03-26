from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient
from pontus_autonomy.tasks.base_task import BaseTask


class GateToVertical(BaseTask):
    def __init__(self):
        super().__init__("gate_to_vertical")
        # Hyperparameters

        # Denotes how much forward the sub should go before attempting the vertical marker
        self.forward_distance = 7.0
        # End
        self.odom_sub = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odom_callback,
            10,
        )

        self.current_pose = Pose()
        self.timer = self.create_timer(
            0.2, self.go
        )

        self.go_to_pose_client = GoToPoseClient(self)

        self.cmd_sent = False

    # Callbacks
    def odom_callback(self, msg: Odometry) -> None:
        """
        Handle callback for odometry.

        This is used to get the new location to find the vertical marker.

        Args:
        ----
        msg (Odometry): the odometry message

        Return:
        ------
        None

        """
        self.current_pose = msg.pose.pose

    # Autonomy
    def go(self) -> None:
        """
        Command AUV to go to next point to find vertical marker.

        Args:
        ----
        None

        Return:
        ------
        None

        """
        if not self.current_pose:
            self.get_logger().info("Waiting for current pose")
            return

        if not self.cmd_sent:
            cmd_pose = Pose()
            cmd_pose.position.x = self.current_pose.position.x + self.forward_distance
            cmd_pose.position.y = self.current_pose.position.y
            cmd_pose.position.z = self.current_pose.position.z
            self.go_to_pose_client.go_to_pose(cmd_pose, True)
            self.cmd_sent = True
        elif self.cmd_sent and self.go_to_pose_client.at_pose():
            self.complete(True)
