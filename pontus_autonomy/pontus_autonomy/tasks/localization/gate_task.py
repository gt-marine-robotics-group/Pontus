from enum import Enum
import numpy as np
from typing import Optional
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import rclpy.client
import tf_transformations
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj
from pontus_msgs.srv import GetGateLocation
from pontus_msgs.srv import GateInformation
from pontus_msgs.msg import YOLOResultArray


class GateTask(BaseTask):
    class State(Enum):
        Searching = 0
        Passing_Through = 1
        Done = 2

    class SearchState(Enum):
        Turn_CCW = 0
        Turn_CC = 1
        Detect_Left = 2
        Detect_Right = 3
        Done = 4

    def __init__(self):
        super().__init__("Gate_Task")

        # Hyperparameters / hardcoded values

        # This represents how much the sub turns to identify the left / right gate
        self.search_angle = np.pi/4

        # End

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

        self.yolo_subscription = self.create_subscription(
            YOLOResultArray,
            '/pontus/camera_2/yolo_results',
            self.yolo_callback,
            10
        )

        self.camera_info_left_subscriber = self.create_subscription(
            CameraInfo,
            '/pontus/camera_2/camera_info',
            self.camera_info_callback_left,
            10,
        )

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
        self.searching_state = self.SearchState.Turn_CCW
        self.previous_state = None
        self.desired_depth = None
        self.detected = False
        self.current_pose = None
        self.sent = False
        self.yolo_detections = None
        self.Tx = None
        self.cx = None
        self.cy = None
        self.image_width = None
        self.previous_searching_state = self.SearchState.Turn_CCW
        self.current_yaw = 0.0

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
        # On the first call, maintain this pose
        if self.current_pose is None:
            self.desired_depth = msg.pose.pose.position.z
        self.current_pose = msg.pose.pose
        quat = [self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w]
        _, _, self.current_yaw = tf_transformations.euler_from_quaternion(quat)

    def yolo_callback(self, msg: YOLOResultArray) -> None:
        """
        Handle yolo callback.

        Args:
        ----
        msg (YOLOResultArray): the yolo detections

        Return:
        ------
        None

        """
        self.yolo_detections = msg

    def camera_info_callback_left(self, msg: CameraInfo) -> None:
        """
        Handle callback for camera info of left camera.

        The left camera info topics will be used for calculating the x and y location
        of the object in the camera frame.

        Args:
        ----
        msg (CameraInfo): camera info message from topic

        Return:
        ------
        None

        """
        self.f = msg.k[0]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.image_width = msg.width

    # Helpers
    def get_gate_location(self,
                          detection_client: rclpy.client.Client
                          ) -> tuple[Optional[Point], Optional[Point]]:
        """
        Return the gate detection.

        If the left/right detection is not detected, the respective value will be None.

        Args:
        ----
        detection_client (rclpy.client.Client): the gate detection client

        Return:
        ------
        tuple[Optional[Point], Optional[Point]]: a tuple containing the left and right gate
                                                 detection if the resepctive gate is found

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

    def side_in_view(self, left: bool) -> bool:
        """
        Return whether or not we have a yolo detection of the left gate in view.

        Args:
        ----
        left (bool): if we want to see if the left or right side is in view

        Return:
        ------
        bool: whether the left gate is currently detected

        """
        if self.yolo_detections is None:
            return False
        desired_id = 0 if left else 1
        for detection in self.yolo_detections.results:
            if detection.class_id == desired_id:
                return True
        return False

    def get_angle(self, left_gate: bool) -> float:
        """
        Get the angle of a gate detection.

        Args:
        ----
        left_gate (bool): whether to get the angle of the left gate or right gate
                          if true, will return the left gate

        Return:
        ------
        float: angle of the gate

        """
        desired_id = 0 if left_gate else 1
        for detection in self.yolo_detections.results:
            if detection.class_id == desired_id:
                x = (detection.x1 + detection.x2) / 2
                x_norm = (self.cx - x) / self.f
                angle = np.arctan2(x_norm, 1)
                return angle

    # Autonomy
    def state_machine(self) -> None:
        """
        Perform state machine for autonomy.

        The state machine to do the gate task. The state machine is defined as the following:
            1. Search for the gate by turning left 45 degrees and right 45 degrees to detect
               both sides of the gate.
               TODO: Create autonomy that handles not seeing the gate when turned
            2. Go to the center point of the detected gate
            3. Done, saves the gate location and side we came through. And completes the task
               future

        Args:
        ----
        None

        Return:
        ------
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
            self.go_to_pose_client.go_to_pose(cmd_pose.cmd_pose, cmd_pose.skip)

    def search(self) -> Optional[PoseObj]:
        """
        Find the gate by turning.

        Logic:
            1. First turn to 0 degrees to start our search
            2. Continue to turn clockwise
            3. If our yolo model detects a left or right gate, center ourselves with the detection
               and wait 2 seconds to allow the depth estimation
            4. Done

        Since our IMU isnt calibrated yet, we will just assume that we have to turn 90 degrees first
        TODO: Tune IMU, adjust logic

        Args:
        ----
        None

        Return:
        ------
        Optional[PoseObj]: if None, indicates the sub should maintain its current trajectory,
                           else indicates the new pose the sub should go to

        """
        match self.searching_state:
            case self.SearchState.Turn_CCW:
                return self.turn()
            case self.SearchState.Turn_CC:
                return self.turn()
            case self.SearchState.Detect_Left:
                return self.align_with_gate(True)
            case self.SearchState.Detect_Right:
                return self.align_with_gate(False)
            case self.SearchState.Done:
                self.state = self.State.Passing_Through

    def turn(self) -> Optional[Pose]:
        """
        Turn to find the gate.

        This will transition to the next state once the semantic map sees both.

        Cases:
            1. See both, end
            2. See left, handle
            3. See right, handle
            4. See neither + at the end of turning CCW -> turn CC
            5. Not see Left and not see right, continue turning
            6. Seen right but not left, turn CCW
            7. Seen right + at the end of turning CCW -> handle
            8. Not see right but found left -> turn CCW
            9. See left but not right + at the end of turning CC -> Handle
            10. See neither + at end of turning CC -> handle

        TODO: Handle situation #7, #9, #10

        Args:
        ----
        None

        Return:
        ------
        Optional[Pose]: command pose

        """
        # TODO: Make these use the IMU
        start = np.pi / 3
        end = -np.pi / 3
        left_gate, right_gate = self.get_gate_location(self.gate_detection_client)
        # Case 1
        if left_gate and right_gate:
            self.searching_state = self.SearchState.Done
            self.sent = False
            return
        # Case 2
        elif not left_gate and self.side_in_view(True):
            self.previous_searching_state = self.searching_state
            self.searching_state = self.SearchState.Detect_Left
            self.sent = False
        # Case 3
        elif not right_gate and self.side_in_view(False):
            self.previous_searching_state = self.searching_state
            self.searching_state = self.SearchState.Detect_Right
            self.sent = False
        # Case 4
        elif not left_gate and not right_gate and abs(start - self.current_yaw) < 0.05:
            self.searching_state = self.SearchState.Turn_CC
            self.sent = False
            return
        # Case 5
        elif not left_gate and not right_gate and not self.sent:
            cmd_pose = Pose()
            desired_angle = start if self.searching_state == self.SearchState.Turn_CCW else end
            quat = tf_transformations.quaternion_from_euler(0, 0, desired_angle)
            cmd_pose.position.z = self.desired_depth
            cmd_pose.orientation.x = quat[0]
            cmd_pose.orientation.y = quat[1]
            cmd_pose.orientation.z = quat[2]
            cmd_pose.orientation.w = quat[3]
            self.sent = True
            return PoseObj(cmd_pose, False)
        # Case 6
        elif not left_gate and right_gate and not self.sent:
            cmd_pose = Pose()
            desired_angle = start
            quat = tf_transformations.quaternion_from_euler(0, 0, desired_angle)
            cmd_pose.position.z = self.desired_depth
            cmd_pose.orientation.x = quat[0]
            cmd_pose.orientation.y = quat[1]
            cmd_pose.orientation.z = quat[2]
            cmd_pose.orientation.w = quat[3]
            self.sent = True
            return PoseObj(cmd_pose, False)
        # Case 7
        # TODO: Handle this
        elif not left_gate and right_gate and abs(end - self.current_yaw) < 0.05:
            self.get_logger().warn("Unhandled case where fully turned counter clockwise but "
                                   + "have not found left_gate")
        # Case 8
        elif left_gate and not right_gate and not self.sent:
            cmd_pose = Pose()
            desired_angle = end
            quat = tf_transformations.quaternion_from_euler(0, 0, desired_angle)
            cmd_pose.position.z = self.desired_depth
            cmd_pose.orientation.x = quat[0]
            cmd_pose.orientation.y = quat[1]
            cmd_pose.orientation.z = quat[2]
            cmd_pose.orientation.w = quat[3]
            self.sent = True
            return PoseObj(cmd_pose, False)
        # Case 9
        elif left_gate and not right_gate and abs(end - self.current_yaw) < 0.05:
            self.get_logger().warn("Unhandled case where fully turned clockwise but "
                                   + "have not found right_gate")
        # Case 10
        elif not left_gate and not right_gate and abs(end - self.current_yaw) < 0.05:
            self.get_logger().warn("Unhandled case where at end of search but no gate found")

    def align_with_gate(self, left: bool) -> Optional[PoseObj]:
        """
        Align the sub with the detected gate side.

        This will wait one second to allow for detections.

        Args:
        ----
        left (bool): whether to align with the left or right gate

        Return:
        ------
        Optional[PoseObj]: command pose

        """
        if not self.sent:
            angle = self.get_angle(left)
            if angle is None:
                self.searching_state = self.previous_searching_state
                return
            cmd_pose = Pose()
            cmd_pose.position.z = self.desired_depth
            quat = [self.current_pose.orientation.x,
                    self.current_pose.orientation.y,
                    self.current_pose.orientation.z,
                    self.current_pose.orientation.w]
            _, _, current_yaw = tf_transformations.euler_from_quaternion(quat)
            new_yaw = current_yaw + angle
            quat = tf_transformations.quaternion_from_euler(0.0, 0.0, new_yaw)
            cmd_pose.orientation.x = quat[0]
            cmd_pose.orientation.y = quat[1]
            cmd_pose.orientation.z = quat[2]
            cmd_pose.orientation.w = quat[3]
            self.sent = True
            return PoseObj(cmd_pose, False)

        if self.go_to_pose_client.at_pose():
            # Wait to allow detection
            time.sleep(2)
            self.searching_state = self.previous_searching_state
            self.sent = False
        return

    def pass_through(self) -> Optional[Pose]:
        """
        Given the gate detection from the semantic map, go to the midpoint.

        Args:
        ----
        None

        Return:
        ------
        Optional[PoseObj]: if None, indicates the sub should maintain its current trajectory,
                           else indicates the new pose the sub should go to

        """
        if not self.sent:
            left_gate, right_gate = self.get_gate_location(self.gate_detection_client)
            cmd_pose = Pose()
            cmd_pose.position.x = (left_gate.x + right_gate.x)/2
            cmd_pose.position.y = (left_gate.y + right_gate.y)/2
            cmd_pose.position.z = self.desired_depth
            self.sent = True
            return PoseObj(cmd_pose, True)

        if self.go_to_pose_client.at_pose():
            self.state = self.State.Done
        return None

    def done(self) -> None:
        """
        Finish state machine.

        Calls the service to save information to /pontus/gate_information.
        This is temporary and realistically should go into mapping. This will also complete
        the future to move onto the next task.

        TODO: Move this over to the semantic map

        Args:
        ----
        None

        Return:
        ------
        None

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
