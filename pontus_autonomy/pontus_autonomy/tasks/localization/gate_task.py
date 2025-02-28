from enum import Enum
import numpy as np

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import tf_transformations

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient
from pontus_msgs.srv import GetGateLocation
from pontus_msgs.srv import GateInformation

class State(Enum):
    Searching = 0
    Done = 1


class GateTask(BaseTask):
    def __init__(self):
        super().__init__("Gate_Task")

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

        self.state = State.Searching
        self.previous_state = None
        self.desired_depth = None
        self.autonomy_loop = self.create_timer(
            0.2,
            self.state_machine
        )
        self.detected = False
        self.current_pose = None


    def state_debugger(self):
        if self.previous_state != self.state:
            self.get_logger().info(f"Now at: {self.state.name}")
            self.previous_state = self.state
    
    # Callbacks
    def odom_callback(self, msg: Odometry):
        # On the first call, maintain this pose
        if self.current_pose is None:
            self.desired_depth = msg.pose.pose.position.z
        self.current_pose = msg.pose.pose

    # Helpers
    def get_gate_location(self):
        request = GetGateLocation.Request()
        future = self.gate_detection_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        left_gate = None
        right_gate = None
        if future.result() is not None:
            response = future.result()
            if response.found:
                left_gate = response.left_location
                right_gate = response.right_location
        self.get_logger().debug(f"{left_gate}, {right_gate}")
        return left_gate, right_gate

    # Autonomy
    def state_machine(self):
        if self.current_pose is None or self.desired_depth is None:
            self.get_logger().info("Waiting for current pose update")
            return
        
        self.state_debugger()
        cmd_pose = None
        match self.state:
            case State.Searching:
                cmd_pose = self.search()
            case State.Done:
                self.done()
            case _:
                self.get_logger().info("Unrecognized state")

        if cmd_pose:
            self.go_to_pose_client.go_to_pose(cmd_pose)

    
    # TODO: Make this more robust
    def search(self):
        cmd_pose = Pose()
        # For now, we assume that we can see it
        if not self.detected:
            left_gate, right_gate = self.get_gate_location()
            # TODO: Handle this case
            if left_gate is None or right_gate is None:
                self.get_logger().info("Unable to find gate")
                return None
            _, _, yaw = tf_transformations.euler_from_quaternion([self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w])
            left_gate_x = left_gate.x * np.cos(yaw) - left_gate.y * np.sin(yaw)
            left_gate_y = left_gate.x * np.sin(yaw) + left_gate.y * np.cos(yaw)
            right_gate_x = right_gate.x * np.cos(yaw) - right_gate.y * np.sin(yaw)
            right_gate_y = right_gate.x * np.sin(yaw) + right_gate.y * np.cos(yaw)
            # TODO: Remove the 0.5
            cmd_pose.position.x = self.current_pose.position.x + (left_gate_x + right_gate_x)/2 + 1.0 * np.cos(yaw)
            cmd_pose.position.y = self.current_pose.position.y + (left_gate_y + right_gate_y)/2
            cmd_pose.position.z = self.desired_depth
            # Denotes that we should just keep our orientation
            cmd_pose.orientation.x = -1.0
            self.detected = True
            return cmd_pose
        
        elif self.detected and self.go_to_pose_client.at_pose():
            self.state = State.Done
        return None


    def done(self):
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


        