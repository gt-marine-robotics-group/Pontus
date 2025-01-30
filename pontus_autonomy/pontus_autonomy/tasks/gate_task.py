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
        self.state = State.Searching
        
        self.num_frames_without_gate = 0
        self.pre_love_tap_time = None
        self.timer = self.create_timer(0.1, self.state_machine_callback)
        self.gate_offset = 0.0

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

        self.yolo_sub_right = self.create_subscription(
            YOLOResultArray,
            '/pontus/camera_3/yolo_results',
            self.yolo_results_right_callback,
            10,
        )


        self.yolo_results_left = YOLOResultArray()
        self.yolo_results_right = YOLOResultArray()
        

    def state_debugger(self):
        if self.previous_state != self.state:
            self.get_logger().info(f"Now at: {self.state.name}")
            self.previous_state = self.state
    
    #### Callbacks
    def yolo_results_left_callback(self, msg):
        self.yolo_results_left = msg


    def yolo_results_right_callback(self, msg):
        self.yolo_results_right = msg


    #### Helpers
    def gate_detection_cv(self):
        left_gate_location_left_camera = None
        right_gate_location_left_camera = None

        for result in self.yolo_results_left.results:
            if result.class_id == 0:
                left_gate_location_left_camera = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])
            if result.class_id == 1:
                right_gate_location_left_camera = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])
        return left_gate_location_left_camera, right_gate_location_left_camera
    
    
    def calculate_stereo_location(self):
        left_gate_location_left_camera = None
        right_gate_location_left_camera = None

        left_gate_location_right_camera = None
        right_gate_location_right_camera = None
        # Find the middle point of the gate
        for result in self.yolo_results_left.results:
            if result.class_id == 0:
                left_gate_location_left_camera = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])
            if result.class_id == 1:
                right_gate_location_left_camera = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])
        
        for result in self.yolo_results_right.results:
            if result.class_id == 0:
                left_gate_location_right_camera = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])
            if result.class_id == 1:
                right_gate_location_right_camera = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])

        focal_length = 381.3
        hfov = 1.39626
        d = 0.075
        left_gate_location = None
        right_gate_location = None
        offset_angle = None
        # If all components are detected, run full stereo detection algorithm
        if left_gate_location_left_camera is not None and left_gate_location_right_camera is not None \
            and right_gate_location_left_camera is not None and right_gate_location_right_camera is not None:
            # Calculate left gate disparity
            disparity_left = left_gate_location_left_camera[0] - left_gate_location_right_camera[0] 
            # Calculate left gate depth
            left_depth = focal_length * d / disparity_left
            # Calculate left gate and right gate x coordinate
            left_gate_angle_wrt_left = left_gate_location_left_camera[0] / self.image_width * hfov - hfov / 2
            right_gate_angle_wrt_left = right_gate_location_left_camera[0] / self.image_width * hfov  - hfov / 2
            left_x = left_depth * np.tan(left_gate_angle_wrt_left) + d / 2
            right_x = left_depth * np.tan(right_gate_angle_wrt_left) + d/2 
            
            # Calculate right gate disparity
            disparity_right = right_gate_location_left_camera[0]  - right_gate_location_right_camera[0]
            # Calculate right gate depth
            right_depth = focal_length * d / disparity_right
            left_gate_location = np.array([left_x, left_depth])
            right_gate_location = np.array([right_x, right_depth])

            offset_angle = np.arctan2((right_x - left_x), (right_depth - left_depth)) - np.pi / 2

        return left_gate_location, right_gate_location, offset_angle


    def calculate_gate_fov_bound(self, left_gate, right_gate):
        distance = 1
        if left_gate is not None and right_gate is not None:
            distance = (right_gate[0] - left_gate[0]) / self.image_width
        return distance

    # def calculate_stereo_location(self):
    #     left_gate_location_left_camera = None
    #     right_gate_location_left_camera = None

    #     left_gate_location_right_camera = None
    #     right_gate_location_right_camera = None
    #     # Find the middle point of the gate
    #     for result in self.yolo_results_left.results:
    #         if result.class_id == 0:
    #             left_gate_location_left_camera = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])
    #         if result.class_id == 1:
    #             right_gate_location_left_camera = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])
        
    #     for result in self.yolo_results_right.results:
    #         if result.class_id == 0:
    #             left_gate_location_right_camera = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])
    #         if result.class_id == 1:
    #             right_gate_location_right_camera = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])
    #     distance_between_gates = 1
    #     if left_gate_location_left_camera is not None and right_gate_location_left_camera is not None:
    #         distance_between_gates = (right_gate_location_left_camera[0] - left_gate_location_left_camera[0]) / self.image_width

    #     left_depth = None
    #     right_depth = None
    #     focal_length = 381.3
    #     hfov = 1.39626
    #     d = 0.075
    #     # If all components are detected, run full stereo detection algorithm
    #     # This is important to calculate the angle offset between us and the gate    
    #     if left_gate_location_left_camera is not None and left_gate_location_right_camera is not None \
    #         and right_gate_location_left_camera is not None and right_gate_location_right_camera is not None \
    #             and distance_between_gates < 0.8:
    #         # Calculate left gate disparity
    #         disparity_left = left_gate_location_left_camera[0] - left_gate_location_right_camera[0] 
    #         # Calculate left gate depth
    #         left_depth = focal_length * d / disparity_left
    #         # Calculate left gate and right gate x coordinate
    #         left_gate_angle_wrt_left = left_gate_location_left_camera[0] / self.image_width * hfov - hfov / 2
    #         right_gate_angle_wrt_left = right_gate_location_left_camera[0] / self.image_width * hfov  - hfov / 2
    #         left_x = left_depth * np.tan(left_gate_angle_wrt_left) + d / 2
    #         right_x = left_depth * np.tan(right_gate_angle_wrt_left) + d/2 
            
    #         # Calculate right gate disparity
    #         disparity_right = right_gate_location_left_camera[0]  - right_gate_location_right_camera[0]
    #         # Calculate right gate depth
    #         right_depth = focal_length * d / disparity_right


    #         offset_angle = np.arctan2((right_x - left_x), (right_depth - left_depth)) - np.pi / 2
    #         self.get_logger().info(f"Left x {left_x} Right x {right_x} Left depth {left_depth} Right depth {right_depth}")
    #         self.get_logger().info(f"Offset angle: {offset_angle}")

    #     # self.get_logger().info(f"{left_depth} {right_depth}")
    #     # self.get_logger().info(f"{left_gate_location_left_camera} {left_gate_location_right_camera}")
    #     return left_gate_location_left_camera, right_gate_location_left_camera, distance_between_gates
    #     # return None, None, distance_between_gates


    #### Autonomy
    def state_machine_callback(self):
        self.state_debugger()
        cmd_vel = Twist()
        match self.state:
            case State.Searching:
                cmd_vel = self.search()
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
        cmd_vel = Twist()
        left_gate_location, right_gate_location = self.gate_detection_cv()
        # TODO: Handle the case where our CV model doesnt see the gate, for now it will do nothing
        if left_gate_location is None or right_gate_location is None:
            return cmd_vel
        # Calculate the midpoint
        mid_point = (left_gate_location[0] + right_gate_location[0]) / 2
        # Calculate how much off we are from the middle
        angular_offset = (self.image_width // 2 - mid_point) / self.image_width
        self.get_logger().info(f"{angular_offset}")
        # Command to go in that direction
        cmd_vel.angular.z = self.angular_correction_factor * angular_offset
        if abs(angular_offset) < 0.01:
            self.state = State.Approaching
            for _ in range(5):
                _, _, offset = self.calculate_stereo_location()
                if offset is not None:
                    self.gate_offset = offset
                    break
            else:
                # TODO: Handle this situation
                self.get_logger().info("Failed to calculate stereo position of gates")
            
        return cmd_vel
    
    # TODO: Make this more robust
    def approach(self):
        cmd_vel = Twist()
        left_gate_location, right_gate_location = self.gate_detection_cv()
        # TODO: Handle when we dont see the gate
        if left_gate_location is None or right_gate_location is None:
            return cmd_vel
        mid_point = (left_gate_location[0] + right_gate_location[0]) / 2
        
        # If we are far too left, this will be a negative value
        # If we are far too right, this will be a positive value
        angular_offset = (self.image_width // 2 - mid_point) / self.image_width
        cmd_vel.angular.z = self.angular_correction_factor * angular_offset
        cmd_vel.linear.x = 0.5
        gate_fov_bound = self.calculate_gate_fov_bound(left_gate_location, right_gate_location)
        # The idea here, is that the closer we get to the gates, they will move closer towards the bounds of our FOV
        if gate_fov_bound > 0.85:
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

