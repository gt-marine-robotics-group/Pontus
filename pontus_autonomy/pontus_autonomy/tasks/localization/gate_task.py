from enum import Enum
import numpy as np
import copy

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

from pontus_autonomy.tasks.base_task import BaseTask
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
        self.gate_detection_client = self.create_client(
            GetGateLocation,
            '/pontus/get_gate_detection',
            callback_group=self.service_callback_group
        )
        while not self.gate_detection_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get gate location service")
        
        self.gate_information_client = self.create_client(
            GateInformation,
            '/pontus/gate_information',
            callback_group=self.service_callback_group
        )
        while not self.gate_information_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting to connect to /pontus/gate_information service")


        self.cmd_pose_pub = self.create_publisher(
            Pose,
            '/cmd_pos',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odom_callback,
            10
        )

        self.position_controller_state = self.create_subscription(
            Int32,
            '/pontus/position_controller_state_machine_status',
            self.position_controller_state_callback,
            10
        )

        self.state = State.Searching
        self.previous_state = None

        self.autonomy_loop = self.create_timer(
            0.2,
            self.state_machine
        )
        self.detected = False
        self.current_pose = None
        self.previous_command_pose = Pose()
        self.started = False
        self._done = False


    def state_debugger(self):
        if self.previous_state != self.state:
            self.get_logger().info(f"Now at: {self.state.name}")
            self.previous_state = self.state
    
    # Callbacks
    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
    
    def position_controller_state_callback(self, msg: Int32):
        if not self.started and msg.data > 1:
            self.started = True
        if self.started and msg.data == 1:
            self._done = True
            self.started = False

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
        if self.current_pose is None:
            self.get_logger().info("Waiting for current pose update")
            return
        self.state_debugger()
        cmd_pose = self.current_pose
        match self.state:
            case State.Searching:
                cmd_pose = self.search()
            case State.Done:
                self.done()
            case _:
                self.get_logger().info({"Unrecognized state"})
        cmd_pose.position.z = -1.2
        # self.get_logger().info(f"{cmd_pose} {self.previous_command_pose}")
        if np.linalg.norm(np.array([
            cmd_pose.position.x - self.previous_command_pose.position.x,
            cmd_pose.position.y - self.previous_command_pose.position.y,
            cmd_pose.position.z - self.previous_command_pose.position.z]
        )) > 0.3:
            self.cmd_pose_pub.publish(cmd_pose)
            self.previous_command_pose = cmd_pose

    
    # TODO: Make this more robust
    def search(self):
        cmd_pose = Pose()
        cmd_pose.position.x = self.previous_command_pose.position.x
        cmd_pose.position.y = self.previous_command_pose.position.y
        cmd_pose.position.z = self.previous_command_pose.position.z
        cmd_pose.orientation.x = self.previous_command_pose.orientation.x
        cmd_pose.orientation.y = self.previous_command_pose.orientation.y
        cmd_pose.orientation.z = self.previous_command_pose.orientation.z
        cmd_pose.orientation.w = self.previous_command_pose.orientation.w
        # For now, we assume that we can see it
        if not self.detected and self.current_pose is not None:
            left_gate, right_gate = self.get_gate_location()
            # TODO: Handle this case
            if left_gate is None or right_gate is None:
                self.get_logger().info("Unable to find gate")
                return cmd_pose
            cmd_pose.position.x = self.current_pose.position.x + (left_gate.x + right_gate.x)/2
            cmd_pose.position.y = self.current_pose.position.y + (left_gate.y + right_gate.y)/2
            self.detected = True
        elif self.detected and self._done:
            self.state = State.Done
        
        return cmd_pose

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
        return


        