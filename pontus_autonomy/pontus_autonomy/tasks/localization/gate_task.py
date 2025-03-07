from enum import Enum
import numpy as np
from typing import Optional

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import rclpy.client
import tf_transformations
from geometry_msgs.msg import Point

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient
from pontus_msgs.srv import GetGateLocation
from pontus_msgs.srv import GateInformation

class GateTask(BaseTask):
    class State(Enum):
        Searching = 0
        Passing_Through = 1
        Done = 2

    class SearchState(Enum):
        Find_Left = 0
        Turn_Left = 1
        FindRight = 2
        Turn_Right = 3

    def __init__(self):
        super().__init__("Gate_Task")

        ### Hyperparameters / hardcoded values

        # This represents how much the sub turns to identify the left / right gate
        self.search_angle = np.pi/4
        
        ###


        # Need this to prevent deadlock issues
        self.service_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Service to detect gate
        self.gate_detection_client = self.create_client(
            GetGateLocation,
            '/pontus/get_gate_detection',
            callback_group=self.service_callback_group
        )
        while not self.gate_detection_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get gate location service")
        
        # Service to keep track of where the gate is, and which side we went through
        self.gate_information_client = self.create_client(
            GateInformation,
            '/pontus/gate_information',
            callback_group=self.service_callback_group
        )
        while not self.gate_information_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting to connect to /pontus/gate_information service")
        
        # Abstracted go to pose client
        self.go_to_pose_client = GoToPoseClient(self)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odom_callback,
            10
        )
        self.autonomy_loop = self.create_timer(
            0.2,
            self.state_machine
        )

        self.state = self.State.Searching
        self.searching_state = self.SearchState.Find_Left
        self.previous_state = None
        self.desired_depth = None
        self.detected = False
        self.current_pose = None
        self.left_gate = None
        self.right_gate = None
        self.sent = False


    def state_debugger(self) -> None:
        """
        Displays state changes in the state machine

        Parameters:
        None

        Returns:
        None
        """
        if self.previous_state != self.state:
            self.get_logger().info(f"Now at: {self.state.name}")
            self.previous_state = self.state
    

    # Callbacks
    def odom_callback(self, msg: Odometry) -> None:
        """
        Keeps track of current odometry. On the first odometry message, will set the desired depth
        as the current depth.

        Parameters:
        msg (Odometry) : odometry message from /pontus/odometry

        Returns:
        None
        """
        # On the first call, maintain this pose
        if self.current_pose is None:
            self.desired_depth = msg.pose.pose.position.z
        self.current_pose = msg.pose.pose


    # Helpers
    def get_gate_location(self, detection_client: rclpy.client.Client) -> tuple[Optional[Point], Optional[Point]]:
        """
        Returns the gate detection. If the left/right detection is not detected, the respective value
        will be None.

        Parameters:
        detection_client (rclpy.client.Client) : the gate detection client

        Returns:
        tuple[Optional[Point], Optional[Point]] : a tuple containing the left and right gate detection if the 
                                                  resepctive gate is found
        """
        request = GetGateLocation.Request()
        future = detection_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        left_gate = None
        right_gate = None
        if future.result() is not None:
            response = future.result()
            if response.left_valid:
                left_gate = response.left_location
            if response.right_valid:
                right_gate = response.right_location
        return left_gate, right_gate


    # Autonomy
    def state_machine(self) -> None:
        """
        The state machine to do the gate task. The state machine is defined as the following:
            1. Search for the gate by turning left 45 degrees and right 45 degrees to detect both sides of the gate.
               TODO: Create autonomy that handles not seeing the gate when turned
            2. Go to the center point of the detected gate
            3. Done, saves the gate location and side we came through. And completes the task future
        
        Parameters:
        None

        Returns:
        None
        """
        if self.current_pose is None or self.desired_depth is None:
            self.get_logger().info("Waiting for current pose update")
            return
        
        self.state_debugger()
        cmd_pose = None
        match self.state:
            case self.State.Searching:
                cmd_pose = self.search()
            case self.State.Passing_Through:
                cmd_pose = self.pass_through()
            case self.State.Done:
                self.done()
            case _:
                self.get_logger().info("Unrecognized state")

        if cmd_pose:
            self.go_to_pose_client.go_to_pose(cmd_pose)


    def search(self) -> Optional[Pose]:
        """
        Find the gate by first turning left to detect the left gate. Then turn right to detect the right gate

        Parameters:
        None

        Returns:
        Optional[Pose] : if None, indicates the sub should maintain its current trajectory,
                         else indicates the new pose the sub should go to
        """
        cmd_pose = self.current_pose
        self.left_gate, self.right_gate = self.get_gate_location(self.gate_detection_client)
        GREEN = "\033[92m"
        RESET = "\033[0m"
        YELLOW = "\033[93m"

        # If both gates are found, transition to next state
        if self.left_gate and self.right_gate:
            self.get_logger().info(f"{GREEN} Found gate! {RESET} {self.left_gate} {self.right_gate}")
            surprise = """╭━━━╮┈┈╱╲┈┈┈╱╲\n┃╭━━╯┈┈▏▔▔▔▔▔▏\n┃╰━━━━━▏╭▆┊╭▆▕\n╰┫╯╯╯╯╯▏╰╯▼╰╯▕\n┈┃╯╯╯╯╯▏╰━┻━╯▕\n┈╰┓┏┳━┓┏┳┳━━━━╯\n┈┈┃┃┃┈┃┃┃┃┈┈┈┈\n┈┈┗┻┛┈┗┛┗┛┈┈┈┈"""
            self.get_logger().info(f"\n{surprise}")
            self.state = self.State.Passing_Through
            return cmd_pose
        
        # TODO: Make this following logic better
        # If left_gate found, turn to where we expect the right_gate to be,
        # For now, just do 45 degrees
        if self.left_gate:
            self.get_logger().info(f"{YELLOW} Found left gate: {RESET} {self.left_gate}")
            quat = tf_transformations.quaternion_from_euler(0, 0, -self.search_angle)
            cmd_pose.orientation.x = quat[0]
            cmd_pose.orientation.y = quat[1]
            cmd_pose.orientation.z = quat[2]
            cmd_pose.orientation.w = quat[3]
            self.searching_state = self.SearchState.Turn_Right
            return cmd_pose

        if not self.left_gate and self.searching_state == self.SearchState.Find_Left:
            quat = tf_transformations.quaternion_from_euler(0, 0, self.search_angle)
            cmd_pose.orientation.x = quat[0]
            cmd_pose.orientation.y = quat[1]
            cmd_pose.orientation.z = quat[2]
            cmd_pose.orientation.w = quat[3]
            self.searching_state = self.SearchState.Turn_Left
            return cmd_pose
        
        if self.searching_state == self.SearchState.Turn_Left and not self.go_to_pose_client.at_pose():
            return
        if self.searching_state == self.SearchState.Turn_Left and self.go_to_pose_client.at_pose():
            if not self.left_gate:
                self.get_logger().info("Waiting to get left gate detection")
            return None

        if self.searching_state == self.SearchState.Turn_Right and not self.go_to_pose_client.at_pose():
            return None
        if self.searching_state == self.SearchState.Turn_Right and self.go_to_pose_client.at_pose():
            self.get_logger().info("Waiting to get right gate detection")
        return None

        
    def pass_through(self) -> Optional[Pose]:
        """
        Given the gate detection from the semantic map, go to the midpoint.

        Parameters:
        None

        Returns:
        Optional[Pose] : if None, indicates the sub should maintain its current trajectory,
                         else indicates the new pose the sub should go tos
        """
        if not self.sent:
            cmd_pose = Pose()
            cmd_pose.position.x = (self.left_gate.x + self.right_gate.x)/2
            cmd_pose.position.y = (self.left_gate.y + self.right_gate.y)/2
            cmd_pose.position.z = self.desired_depth
            self.sent = True
            return cmd_pose
        
        if self.go_to_pose_client.at_pose():
            self.state = self.State.Done
        return None


    def done(self) -> None:
        """
        Calls the service to save information to /pontus/gate_information. This is temporary and realistically should go into mapping.
        This will also complete the future to move onto the next task.

        Parameters:
        None

        Returns None
        """
        request = GateInformation.Request()
        request.set.data = True
        request.entered_left_side.data = False
        request.gate_location.x = self.current_pose.position.x
        request.gate_location.y = self.current_pose.position.y
        request.gate_location.z = self.current_pose.position.z
        future = self.gate_information_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.result() is not None:
            self.get_logger().info("Successfully sent information to service")
            self.complete(True)
        return None
