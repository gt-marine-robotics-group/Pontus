from enum import Enum
from typing import Optional

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import rclpy.client
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient
from pontus_autonomy.tasks.base_task import BaseTask
from pontus_msgs.srv import GetVerticalMarkerLocation


class VerticalMarkerTask(BaseTask):
    class State(Enum):
        Searching = 0
        Circumnavigate = 1
        Done = 2

    def __init__(self):
        super().__init__("Vertial_Marker_task")

        # Need this to prevent deadlock issues
        self.service_callback_group = MutuallyExclusiveCallbackGroup()
        self.vm_detection_client = self.create_client(
            GetVerticalMarkerLocation,
            '/pontus/get_vertical_marker_detection',
            callback_group=self.service_callback_group
        )
        while not self.vm_detection_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get vertial marker location service")

        self.odom_sub = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odom_callback,
            10
        )

        self.go_to_pose_client = GoToPoseClient(self)

        self.state = self.State.Searching
        self.previous_state = None

        self.autonomy_loop = self.create_timer(
            0.2,
            self.state_machine
        )
        self.detected = False
        self.current_pose = None
        self.command_sent = False
        self.current_desired_position = 0
        self.desired_depth = None

    def state_debugger(self) -> None:
        """
        Display state changes in the state machine.

        Args:
        ----
        None

        Return:
        ------
        None

        """
        if self.previous_state != self.state:
            self.get_logger().info(f"Now at: {self.state.name}")
            self.previous_state = self.state

    # Callbacks
    def odom_callback(self, msg: Odometry) -> None:
        """
        Handle odom callback.

        Keeps track of current odometry. On the first odometry message, will set the desired depth
        as the current depth.

        Args:
        ----
        msg (Odometry): odometry message from /pontus/odometry

        Return:
        ------
        None

        """
        if self.current_pose is None:
            self.desired_depth = msg.pose.pose.position.z
        self.current_pose = msg.pose.pose

    # Helpers
    def get_vertical_marker_location(self,
                                     detection_client: rclpy.client.Client) -> Optional[Point]:
        """
        Return the vertical marker detection if detected.

        Args:
        ----
        detection_client (rclpy.client.Client): the vertical marker detection client

        Return:
        ------
        Optional[Point]: vertical marker detection if located

        """
        request = GetVerticalMarkerLocation.Request()
        future = detection_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        vertical_marker = None
        if future.result() is not None:
            response = future.result()
            if response.found:
                vertical_marker = response.location
        self.get_logger().debug(f"{vertical_marker}")
        return vertical_marker

    def copy_pose(self, pose: Pose) -> Pose:
        """
        Deep copies the given pose.

        Args:
        ----
        pose (Pose): the pose we want to copy

        Return:
        ------
        Pose: the copied pose

        """
        new_pose = Pose()
        new_pose.position.x = pose.position.x
        new_pose.position.y = pose.position.y
        new_pose.position.z = pose.position.z
        new_pose.orientation.x = pose.orientation.x
        new_pose.orientation.y = pose.orientation.y
        new_pose.orientation.z = pose.orientation.z
        new_pose.orientation.w = pose.orientation.w
        return new_pose

    # Autonomy
    def state_machine(self) -> None:
        """
        Perform state machine for vertical marker task.

        The state machine is defined as the following:
            1. Detect the vertical marker and get its 3D pose
            2. Then cicumnavigate the vertical marker by having a set of keypoints that is
               just a square.
            3. Complete

        Args:
        ----
        None

        Return:
        ------
        None

        """
        if self.current_pose is None:
            self.get_logger().info("Waiting for current pose update")
            return
        self.state_debugger()
        cmd_pose = None
        match self.state:
            case self.State.Searching:
                cmd_pose = self.search()
            case self.State.Circumnavigate:
                cmd_pose = self.circumnavigate()
            case self.State.Done:
                self.complete(True)
            case _:
                self.get_logger().info("Unrecognized state")
        if cmd_pose:
            self.go_to_pose_client.go_to_pose(cmd_pose)

    def search(self) -> None:
        """
        Search for vertical marker.

        This will find the vertical marker by just looking forward. Then it will set us 1.5 meters
        behind the vertical marker to prepare to circumnavigate it.

        Args:
        ----
        None

        Return:
        ------
        None

        """
        cmd_pose = Pose()
        if not self.detected and self.current_pose is not None:
            vertical_marker = self.get_vertical_marker_location(self.vm_detection_client)
            GREEN = "\033[92m"
            RESET = "\033[0m"
            if vertical_marker is None:
                self.get_logger().info("Unable to find vertical marker")
                return None
            self.get_logger().info(f"{GREEN} Found Vertical Marker! {RESET} {vertical_marker}")
            vertical_surprise = """⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⠤⠤⡀⠀⠀⠀⠀⠀⠀⠀⢠⢤⡀⠀⠀⠀⠀⠠⣄⣀⣀⡤⠤⠤⠤⠤⠤⠀⠀⠀⠀⠀⠀⠀\n⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡤⠎⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⠇⠀⠙⡄⠀⠀⠀⠀⠀⠀⠀⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀\n⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣜⣀⣀⣀⣀⢇⠀⠀⠀⠀⠀⠀⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀\n⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢣⠀⠀⠀⠀⠀⠀⠀⢠⠀⠀⠀⢸⠀⠀⠀⠀⠀⠸⡀⠀⠀⠀⠀⠀⠀⢸⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀\n⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢧⡀⠀⠀⠀⠀⢠⠃⠀⠀⢰⠃⠀⠀⠀⠀⠀⠘⡆⠀⠀⠀⠀⠀⠀⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀\n⠀⠀⠀⠀⠀⠀⠀⠀⠀⢰⠋⠓⢤⠈⠦⢄⡠⠴⠃⠀⠀⠀⠘⠂⠀⠀⠀⠀⠀⠀⠁⠀⠀⠀⠀⠀⠀⣠⠤⠤⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀\n⠀⠀⠀⠀⠀⠀⠀⠀⠀⡇⠀⠀⠀⠈⢧⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡴⠋⠀⠀⠀⠘⡆⠀⠀⠀⠀⠀⠀⠀⠀\n⠀⠀⠀⠀⠀⠀⠀⠀⠀⡇⠀⠀⠀⠀⠀⠙⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠏⠀⠀⠀⠀⠀⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀\n⠀⠀⠀⠀⠀⠀⠀⠀⠀⢇⠀⠀⠀⠀⠀⠀⠈⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡴⠁⠀⠀⠀⠀⠀⠀⢠⠇⠀⠀⠀⠀⠀⠀⠀⠀\n⠀⠀⠀⡀⠀⠀⠀⠀⠀⢸⠀⠀⠀⠀⠀⠀⠀⠈⢇⡤⠤⠖⠒⠒⠋⠉⠉⠓⠲⢤⠜⠁⠀⠀⠀⠀⠀⠀⠀⠀⡞⠀⠀⣠⠔⠀⠀⠀⠀⠀\n⠀⠀⠀⠉⠓⠤⢄⣀⢀⡤⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣀⡤⡄⠀⠀⠀⠀⠀⣀⡠⠤⠤⡄⠀⠀⠀⠀⠀⠀⣰⠯⢷⠋⠁⠀⠀⠀⠀⠀\n⠀⠉⠓⠦⣄⡀⠀⢠⠏⠉⠉⠑⠒⠒⠀⠀⠀⠴⠋⠀⠀⠀⠀⠁⠀⠀⠀⡉⠁⠀⠀⠀⠀⠉⠀⠠⠴⠒⠉⠀⠀⠀⡠⠇⠀⢀⣀⡤⠄⠀\n⠀⠀⠀⠀⠀⠈⠑⠺⡤⢄⣀⣀⣀⣀⣀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣉⡷⠾⠥⢴⠤⠤⠄⠀⠀⠀⠀⣠⠤⠤⢤⠞⠛⠉⠈⠁⠀⠀⠀⠀⠀\n⠀⠀⠀⠀⠄⠐⠦⣄⠘⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⠁⠀⣇⠀⠀⠀⡸⠀⠀⠀⠀⠀⠀⠀⠀⠀⢯⠀⠀⢀⡴⠊⠃⠁⢻⠀⠀\n⠀⠀⠀⠀⠲⡀⠀⠀⢑⣏⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⠒⠒⠋⠀⠀⠀⠀⠀⠀⠀⠀⣀⡼⠦⠊⠁⠀⠀⠀⠀⡴⠁⠀\n⠀⠀⠀⠀⠀⠙⡆⠀⠘⠊⠳⠤⠤⠤⠤⠒⠒⠋⠉⠑⢤⣀⠀⣀⡠⠤⠒⠉⠳⠤⠤⠴⠊⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⣀⡇⠀⠀⠀⠀⠀\n⠀⠀⠀⠀⠀⠀⠈⠧⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡀⠀⠀⠀⠀⠀⣀⡠⠤⠚⠁⠀⠀⠀\n⠀⠀⠀⠀⠀⠀⠀⠀⠈⠓⠒⠒⠒⠲⠤⣄⣤⠴⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢷⠒⠒⠒⠒⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀\n⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡖⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢣⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀\n⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀\n⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀\n⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀\n⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡼⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀\n⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀\n⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣏⡠⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢧⠀⠀⠀⠀⠀⠀⠀⠀⠀"""  # noqa E501
            # Convert detection to odom frame
            self.get_logger().info(f"\n{vertical_surprise}")
            cmd_pose.position.x = vertical_marker.x - 1.5
            cmd_pose.position.y = vertical_marker.y
            cmd_pose.position.z = self.desired_depth
            self.detected = True
            return cmd_pose

        elif self.detected and self.go_to_pose_client.at_pose():
            self.state = self.State.Circumnavigate
            self.starting_pose = self.current_pose
            self.get_logger().info(f"Starting pose: {self.starting_pose}")

        return None

    def circumnavigate(self) -> None:
        """
        Circumnavigate the vertical marker.

        This will command the AUV to go in a box around the vertical marker.

        Args:
        ----
        None

        Return:
        ------
        None

        """
        cmd_pose = Pose()

        # First pose
        first_pose = self.copy_pose(self.starting_pose)
        first_pose.position.y += 0.9

        # Second pose
        second_pose = self.copy_pose(self.starting_pose)
        second_pose.position.y += 0.9
        second_pose.position.x += 2.3

        # Third Pose
        third_pose = self.copy_pose(self.starting_pose)
        third_pose.position.y += -0.9
        third_pose.position.x += 2.3

        # Fourth Pose
        fourth_pose = self.copy_pose(self.starting_pose)
        fourth_pose.position.y += -0.9

        desired_positions = [first_pose, second_pose, third_pose, fourth_pose, self.starting_pose]

        if not self.command_sent:
            cmd_pose = desired_positions[self.current_desired_position]
            cmd_pose.orientation.x = -1.0
            self.command_sent = True
            return cmd_pose

        elif self.go_to_pose_client.at_pose():
            self.current_desired_position += 1
            self.command_sent = False

        if self.current_desired_position == len(desired_positions):
            self.state = self.State.Done

        return None
