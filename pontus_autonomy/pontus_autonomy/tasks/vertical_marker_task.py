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


class VerticalMarkerTask(BaseTask):
    def __init__(self):
        super().__init__("Vertical_Task")
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.yolo_sub_left = self.create_subscription(
            YOLOResultArray,
            '/pontus/camera_2/yolo_results',
            self.yolo_results_left_callback,
            10,
        )

        self.previous_state = None
        self.state = State.Searching

        self.yolo_results_left = YOLOResultArray()
        self.image_width = 640
        self.angular_correction_factor = 1.0
    
        self.timer = self.create_timer(0.1, self.state_machine_callback)

    def state_debugger(self):
        if self.previous_state != self.state:
            self.get_logger().info(f"Now at: {self.state.name}")
            self.previous_state = self.state

    def yolo_results_left_callback(self, msg):
        self.yolo_results_left = msg
    

    #### Helpers
    def vertical_detection_cv(self):
        vertical_marker_location = None
        for result in self.yolo_results_left.results:
            if result.class_id == 2:
                vertical_marker_location = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])
        return vertical_marker_location

    #### Autonomy
    def state_machine_callback(self):
        self.state_debugger()
        cmd_vel = Twist()
        match self.state:
            case State.Searching:
                cmd_vel = self.search()
            case State.Approaching:
                cmd_vel = self.approach()
            case _:
                pass
        
        self.cmd_vel_pub.publish(cmd_vel)
    

    def search(self):
        cmd_vel = Twist()
        vertical_marker_location = self.vertical_detection_cv()
        if vertical_marker_location is None:
            self.get_logger().info("Cannot find vertical marker")
            return cmd_vel
        angular_offset = (self.image_width // 2 - vertical_marker_location[0]) / self.image_width
        cmd_vel.angular.z = self.angular_correction_factor * angular_offset
        if abs(angular_offset) < 0.01:
            self.state = State.Approaching
        return cmd_vel


    def approach(self):
        cmd_vel = Twist()
        vertical_marker_location = self.vertical_detection_cv()
        # TODO: Handle this
        if vertical_marker_location is None:
            self.get_logger().info(f"Unable to find vertical marker")
            return cmd_vel
        angular_offset = (self.image_width // 2 - vertical_marker_location[0]) / self.image_width
        cmd_vel.angular.z = self.angular_correction_factor * angular_offset
        cmd_vel.linear.x = 0.5
        return cmd_vel