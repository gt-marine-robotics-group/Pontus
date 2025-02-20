from enum import Enum
import numpy as np
import copy

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_msgs.srv import GetGateLocation


class State(Enum):
    Searching = 0
    Approaching = 1
    Love_Tap = 2
    Orientation_Correction = 3
    Passing_Through = 4
    PassedThrough = 5


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
            pass

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

        self.state = State.Searching
        self.previous_state = None

        self.autonomy_loop = self.create_timer(
            0.2,
            self.state_machine
        )
        self.detected = False
        self.current_pose = None
        self.previous_command_pose = Pose()


    def state_debugger(self):
        if self.previous_state != self.state:
            self.get_logger().info(f"Now at: {self.state.name}")
            self.previous_state = self.state
    
    # Callbacks
    def odom_callback(self, msg: Odometry):
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
        self.state_debugger()
        cmd_pose = Pose()
        match self.state:
            case State.Searching:
                cmd_pose = self.search()
            case _:
                pass
        cmd_pose.position.z = -1.2
        self.get_logger().info(f"{cmd_pose} {self.previous_command_pose}")
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
        # cmd_pose = copy.copy(self.previous_command_pose)
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
        else:
            pass
        
        return cmd_pose


        