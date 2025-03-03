import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_msgs.msg import YOLOResultArray, YOLOResult
from pontus_autonomy.tasks.base_task import BaseTask

from enum import Enum
import numpy as np


class State(Enum):
    Searching = 0
    Approaching = 1
    DroppingMarker = 2

class Cameras(Enum):
    Front = 0
    Down = 1


class BinTask(BaseTask):
    def __init__(self):
        super().__init__("Bin_Task")
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.cmd_grip_pub = self.create_publisher(
            Twist,
            '/cmd_grip',
            10
        )

        self.yolo_sub_left = self.create_subscription(
            YOLOResultArray,
            '/pontus/camera_2/yolo_results',
            self.yolo_results_left_callback,
            10,
        )

        self.yolo_sub_down = self.create_subscription(
            YOLOResultArray,
            '/pontus/camera_1/yolo_results',
            self.yolo_results_down_callback,
            10,
        )

        self.previous_state = None
        self.state = State.Searching

        self.yolo_results_left = YOLOResultArray()
        self.yolo_results_down = YOLOResultArray()
        self.image_width = 640
        self.angular_correction_factor = 1.0
    
        self.timer = self.create_timer(0.1, self.state_machine_callback)

    def state_debugger(self):
        if self.previous_state != self.state:
            self.get_logger().info(f"Now at: {self.state.name}")
            self.previous_state = self.state

    def yolo_results_left_callback(self, msg):
        self.yolo_results_left = msg

    def yolo_results_down_callback(self, msg):
        self.yolo_results_down = msg
    

    #### Helpers
    def bin_detection_cv(self):
        bin_marker_location = None
        for result in self.yolo_results_left.results:
            if result.class_id == 3:
                camera_detecting = Cameras.Front
                bin_marker_location = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])
                return bin_marker_location
        for result in self.yolo_results_down.results:
            if result.class_id == 1:
                camera_detecting = Cameras.Down
                bin_marker_location = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])
        return bin_marker_location

    #### Autonomy
    def state_machine_callback(self):
        self.state_debugger()
        cmd_vel = Twist()
        match self.state:
            case State.Searching:
                cmd_vel = self.search()
            case State.Approaching:
                cmd_vel = self.approach()
            case State.DroppingMarker:
                cmd_vel = self.drop_marker()
            case _:
                pass
        
        self.cmd_vel_pub.publish(cmd_vel)
    

    def search(self):
        cmd_vel = Twist()
        bin_location = self.bin_detection_cv()
        if bin_location is None:
            self.get_logger().info("Cannot find bin")
            return cmd_vel
        angular_offset = (self.image_width // 2 - bin[0]) / self.image_width
        cmd_vel.angular.z = self.angular_correction_factor * angular_offset
        if abs(angular_offset) < 0.01:
            self.state = State.Approaching
        return cmd_vel


    def approach(self):
        cmd_vel = Twist()
        bin_location = self.bin_detection_cv()
        # TODO: Handle this
        if bin_location is None:
            self.get_logger().info(f"Unable to find bin")
            return cmd_vel
        angular_offset = (self.image_width // 2 - bin[0]) / self.image_width
        cmd_vel.angular.z = self.angular_correction_factor * angular_offset
        cmd_vel.linear.x = 0.5
        return cmd_vel
    
    def drop_marker(self):
        cmd_vel = Twist()
