import rclpy
from geometry_msgs.msg import Twist

from enum import Enum
import numpy as np
import time

from pontus_msgs.msg import YOLOResultArray, YOLOResult
from pontus_autonomy.tasks.base_task import BaseTask

class State(Enum):
    Searching = 0
    Approaching = 1
    Love_Tap = 2
    Passing_Through = 3
    PassedThrough = 4

class GateTask(BaseTask):
    def __init__(self):
        super().__init__("Gate_Task")

        # Config Parameters
        self.image_width = 640

        # Hyperparameters
        # This paramter dictates how fast the sub will turn to center itself with the gate
        # A higher value means quicker, a smaller value means slower
        self.angular_correction_factor = 1.0

        # TODO: Realistically this should start at searching, change later
        self.previous_state = None
        self.state = State.Approaching
        
        self.num_frames_without_gate = 0
        self.pre_love_tap_time = None
        self.timer = self.create_timer(0.1, self.state_machine_callback)

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.yolo_sub = self.create_subscription(
            YOLOResultArray,
            '/pontus/camera_0/yolo_results',
            self.yolo_results_callback,
            10,
        )
        self.yolo_results = YOLOResultArray()
        

    def state_debugger(self):
        if self.previous_state != self.state:
            self.get_logger().info(f"Now at: {self.state.name}")
            self.previous_state = self.state
    
    #### Callbacks
    def yolo_results_callback(self, msg):
        self.yolo_results = msg

    #### Autonomy
    def state_machine_callback(self):
        self.state_debugger()
        cmd_vel = Twist()
        match self.state:
            case State.Searching:
                self.search()
            case State.Approaching:
                cmd_vel = self.approach()
            case State.Love_Tap:
                cmd_vel = self.love_tap()
            case State.Passing_Through:
                cmd_vel = self.pass_through()
            case State.PassedThrough:
                self.destroy_node()
            case _:
                pass
        
        self.cmd_vel_pub.publish(cmd_vel)
    
    def search(self):
        self.state = State.Approaching
    
    # TODO: Make this more robust
    def approach(self):
        cmd_vel = Twist()
        left_gate_location = None
        right_gate_location = None
        # Find the middle point of the gate
        for result in self.yolo_results.results:
            if result.class_id == 0:
                left_gate_location = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])
            if result.class_id == 1:
                right_gate_location = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])
        if left_gate_location is None or right_gate_location is None:
            return cmd_vel
        mid_point = (left_gate_location[0] + right_gate_location[0]) / 2
        self.get_logger().info(f"{left_gate_location[0]} {right_gate_location[0]}")
        
        # If we are far too left, this will be a negative value
        # If we are far too right, this will be a positive value
        angular_offset = (self.image_width // 2 - mid_point) / self.image_width
        cmd_vel.angular.z = self.angular_correction_factor * angular_offset
        cmd_vel.linear.x = 0.5

        # The idea here, is that the closer we get to the gates, they will move closer towards the bounds of our FOV
        distance_between_gates = (right_gate_location[0] - left_gate_location[0]) / self.image_width
        if distance_between_gates > 0.85:
            self.state = State.Love_Tap
            self.pre_love_tap_time = self.get_clock().now()
        return cmd_vel


    def love_tap(self):
        # First a little push to get closer to the center of the gate
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5
        if (self.get_clock().now() - self.pre_love_tap_time).nanoseconds // 1e9 > 2:
            self.state = State.Passing_Through
        self.get_logger().info(f"{(self.get_clock().now() - self.pre_love_tap_time).nanoseconds // 1e9}")
        return cmd_vel


    def pass_through(self):
        # First give a little push
        cmd_vel = Twist()
        return cmd_vel

