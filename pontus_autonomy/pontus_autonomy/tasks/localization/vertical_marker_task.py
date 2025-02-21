from enum import Enum
import numpy as np

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_msgs.srv import GetVerticalMarkerLocation

class State(Enum):
    Searching = 0
    Circumnavigate = 1
    Done = 2

class VerticalMarkerTask(BaseTask):
    def __init__(self):
        super().__init__("Vertial_Marker_task")

        # Need this to prevent deadlock issues
        self.service_callback_group = MutuallyExclusiveCallbackGroup()
        self.vertical_marker_detection_client = self.create_client(
            GetVerticalMarkerLocation,
            '/pontus/get_vertical_marker_detection',
            callback_group=self.service_callback_group
        )
        while not self.vertical_marker_detection_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get vertial marker location service")

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
        self.command_send = False
        self.current_desired_position = 0
    
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
            self._done = False
        if self.started and msg.data == 1:
            self._done = True
            self.started = False
    
    # Helpers
    def get_vertical_marker_location(self):
        request = GetVerticalMarkerLocation.Request()
        future = self.vertical_marker_detection_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        vertical_marker = None
        if future.result() is not None:
            response = future.result()
            if response.found:
                vertical_marker = response.location
        self.get_logger().debug(f"{vertical_marker}")
        return vertical_marker

    def copy_pose(self, pose):
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
    def state_machine(self):
        if self.current_pose is None:
            self.get_logger().info("Waiting for current pose update")
            return
        self.state_debugger()
        cmd_pose = self.current_pose
        match self.state:
            case State.Searching:
                cmd_pose = self.search()
            case State.Circumnavigate:
                cmd_pose = self.circumnavigate()
            # case State.Done:
            #     self.done()
            case _:
                self.get_logger().info("Unrecognized state")
        cmd_pose.position.z = -1.2
        # self.get_logger().info(f"{cmd_pose} {self.previous_command_pose}")
        if np.linalg.norm(np.array([
            cmd_pose.position.x - self.previous_command_pose.position.x,
            cmd_pose.position.y - self.previous_command_pose.position.y,
            cmd_pose.position.z - self.previous_command_pose.position.z]
        )) > 0.3:
            self.cmd_pose_pub.publish(cmd_pose)
            self.previous_command_pose = cmd_pose
            self.get_logger().info(f"Command: {cmd_pose}")


    def search(self):
        cmd_pose = Pose()
        cmd_pose = self.copy_pose(self.previous_command_pose)

        if not self.detected and self.current_pose is not None:
            vertical_marker = self.get_vertical_marker_location()
            if vertical_marker is None:
                self.get_logger().info("Unable to find vertical marker")
                return cmd_pose
            self.get_logger().info(f"{vertical_marker}")
            cmd_pose.position.x = self.current_pose.position.x + (vertical_marker.x - 1.0)
            cmd_pose.position.y = self.current_pose.position.y + vertical_marker.y
            self.detected = True
        elif self.detected and self._done:
            self.state = State.Circumnavigate
            self.starting_pose = self.current_pose
            self._done = False
            self.get_logger().info(f"Starting pose: {self.starting_pose}")
        
        return cmd_pose


    def circumnavigate(self):
        cmd_pose = Pose()
        cmd_pose = self.copy_pose(self.previous_command_pose)

        # First pose
        first_pose = self.copy_pose(self.starting_pose)
        first_pose.position.y += 0.9

        # Second pose
        second_pose = self.copy_pose(self.starting_pose)
        second_pose.position.y += 0.9
        second_pose.position.x += 2.0

        # Third Pose
        third_pose = self.copy_pose(self.starting_pose)
        third_pose.position.y += -0.9
        third_pose.position.x += 2.0

        # Fourth Pose
        fourth_pose = self.copy_pose(self.starting_pose)
        fourth_pose.position.y += -0.9

        desired_positions = [first_pose, second_pose, third_pose, fourth_pose]
        # self.get_logger().info(f"Desired positions: {desired_positions}")
        
        if not self._done and not self.command_send:
            cmd_pose = desired_positions[self.current_desired_position]
            self.command_send = True
        elif self._done:
            self.current_desired_position += 1
            self.command_send = False
            self._done = False
        
        if self.current_desired_position == 4:
            self.state = State.Done
        
        return cmd_pose