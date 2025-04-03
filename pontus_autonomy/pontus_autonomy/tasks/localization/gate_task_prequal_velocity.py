from enum import Enum
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
import tf_transformations

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj
from pontus_controller.position_controller import MovementMethod
from pontus_msgs.msg import YOLOResultArray
from pontus_mapping.semantic_map_manager import SemanticObject


class GateTaskPrequalVelocity(BaseTask):
    class State(Enum):
        GateSearchLeft = 0
        GateSearchRight = 1
        GatePassThrough = 2
        GateDone = 3

    def __init__(self):
        super().__init__('gate_task_prequal_velocity')

        self.create_subscription(
            YOLOResultArray,
            '/pontus/camera_2/yolo_results',
            self.yolo_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odometry_callback,
            10
        )

        self.create_subscription(
            CameraInfo,
            '/pontus/camera_2/camera_info',
            self.camera_info_callback,
            10
        )

        self.current_detections = None

        self.create_timer(
            0.2,
            self.autonomy
        )

        self.go_to_pose_client = GoToPoseClient(self)

        self.state = self.State.GateSearchLeft
        self.current_odometry = None
        self.left_angle = None
        self.right_angle = None
        self.cx = None
        self.fx = None
        self.desired_depth = None
        self.sent = False
        self.previous_state = None

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

    def odometry_callback(self, msg: Odometry) -> None:
        """
        Handle odometry callback.

        Args:
        ----
        msg (Odoometry): the odometry message

        Return:
        ------
        None

        """
        if self.current_odometry is None:
            self.desired_depth = msg.pose.pose.position.z
        self.current_odometry = msg.pose.pose

    def yolo_callback(self, msg: YOLOResultArray) -> None:
        """
        Handle storing yolo callback information.

        Args:
        ----
        msg (YOLOResultArray): the yolo results

        Return:
        ------
        None

        """
        self.current_detections = msg

    def camera_info_callback(self, msg: CameraInfo) -> None:
        """
        Handle camera info callback.

        Args:
        ----
        msg (CameraInfo): the camera info message

        Return:
        ------
        None

        """
        self.cx = msg.k[2]
        self.fx = msg.k[0]

    def yolo_contains(self, obj: SemanticObject) -> bool:
        """
        Return if yolo sees the polled yolo object.

        Args:
        ----
        obj (SemanticObject): the yolo detection we want to see

        Return:
        ------
        bool: if the yolo sees the object or not

        """
        for detection in self.current_detections.results:
            if detection.class_id == obj.value:
                return True
        return False

    def get_object_angle(self, obj: SemanticObject) -> float:
        """
        Find and calculate the angle of a specified obj with respect to the sub.

        Args:
        ----
        obj (SemanticObject): the yolo detection we want to find the angle of

        Return:
        ------
        None

        """
        for detection in self.current_detections.results:
            if detection.class_id == obj.value:
                x = (detection.x1 + detection.x2) / 2
                x_norm = (self.cx - x) / self.fx
                angle = np.arctan2(x_norm, 1)
                return angle
        return None

    def autonomy(self) -> None:
        """
        Run autonomy state machine.

        Args:
        ----
        None

        Return:
        ------
        None

        """
        if self.current_detections is None:
            self.get_logger().warn("Have not received yolo detections yet, skipping")
            return

        if self.current_odometry is None:
            self.get_logger().warn("Have not received current odometry, skipping")
            return

        if self.fx is None or self.cx is None:
            self.get_logger().warn("Have not received camera info message, skipping")
            return
        self.state_debugger()
        pose_obj = None
        match self.state:
            case self.State.GateSearchLeft:
                pose_obj = self.search_for_gate_left()
            case self.State.GateSearchRight:
                pose_obj = self.search_for_gate_right()
            case self.State.GatePassThrough:
                pose_obj = self.gate_pass_through()
            case self.State.GateDone:
                self.complete(True)
            case _:
                pass

        if pose_obj is not None:
            self.go_to_pose_client.go_to_pose(pose_obj)

    def search_for_gate_left(self) -> PoseObj:
        """
        Find the angle of the left gate.

        Args:
        ----
        None

        Return:
        ------
        PoseObj: object representing how we should move

        """
        twist = Twist()
        # Turn left until we see the gate
        if self.yolo_contains(SemanticObject.LeftGate):
            quat = [self.current_odometry.orientation.x,
                    self.current_odometry.orientation.y,
                    self.current_odometry.orientation.z,
                    self.current_odometry.orientation.w]
            _, _, current_angle = tf_transformations.euler_from_quaternion(quat)
            object_angle = self.get_object_angle(SemanticObject.LeftGate)
            self.left_angle = current_angle + object_angle
            self.get_logger().info(f"Left_angle: {self.left_angle}")
            self.state = self.State.GateSearchRight
            return PoseObj(cmd_twist=twist,
                           desired_depth=self.desired_depth,
                           movement_method=MovementMethod.VelocityMaintainDepth)
        twist.angular.z = 0.3
        return PoseObj(cmd_twist=twist,
                       desired_depth=self.desired_depth,
                       movement_method=MovementMethod.VelocityMaintainDepth)

    def search_for_gate_right(self) -> PoseObj:
        """
        Find the angle of the right gate.

        Args:
        ----
        None

        Return:
        ------
        PoseObj: object representing how we should move

        """
        twist = Twist()
        # Turn left until we see the gate
        if self.yolo_contains(SemanticObject.RightGate):
            quat = [self.current_odometry.orientation.x,
                    self.current_odometry.orientation.y,
                    self.current_odometry.orientation.z,
                    self.current_odometry.orientation.w]
            _, _, current_angle = tf_transformations.euler_from_quaternion(quat)
            object_angle = self.get_object_angle(SemanticObject.RightGate)
            self.right_angle = current_angle + object_angle
            self.get_logger().info(f"Right_angle: {self.right_angle}")
            self.get_logger().info(f"{current_angle} {object_angle}")
            self.state = self.State.GatePassThrough
            return PoseObj(cmd_twist=twist,
                           desired_depth=self.desired_depth,
                           movement_method=MovementMethod.VelocityMaintainDepth)
        twist.angular.z = -0.3
        return PoseObj(cmd_twist=twist,
                       desired_depth=self.desired_depth,
                       movement_method=MovementMethod.VelocityMaintainDepth)

    def gate_pass_through(self) -> PoseObj:
        """
        Align the sub so that we are going to go through the middle of the gate.

        Args:
        ----
        None

        Return:
        ------
        PoseObj

        """
        twist = Twist()
        if not self.yolo_contains(SemanticObject.LeftGate) \
                and not self.yolo_contains(SemanticObject.RightGate):
            self.state = self.State.GateDone
            return PoseObj(cmd_twist=twist,
                           desired_depth=self.desired_depth,
                           movement_method=MovementMethod.VelocityMaintainDepth)
        twist.linear.x = 0.5
        return PoseObj(cmd_twist=twist,
                       desired_depth=self.desired_depth,
                       desired_heading=(self.left_angle + self.right_angle) / 2,
                       movement_method=MovementMethod.VelocityMaintainDepthHeading)
