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


class VerticalMarkerTaskVelocity(BaseTask):
    class State(Enum):
        VerticalApproach = 0
        VerticalCircumnavigate = 1
        VerticalDone = 2

    def __init__(self):
        super().__init__('vertical_task_prequal_velocity')

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

        self.state = self.State.VerticalApproach
        self.current_odometry = None
        self.left_angle = None
        self.right_angle = None
        self.cx = None
        self.fx = None
        self.desired_depth = None
        self.sent = False

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

        pose_obj = None
        match self.state:
            case self.State.VerticalApproach:
                pose_obj = self.approach_vertical()
            case _:
                pass

        if pose_obj is not None:
            self.go_to_pose_client.go_to_pose(pose_obj)

    def approach_vertical(self) -> PoseObj:
        """
        Approach the vertical marker.

        Args:
        ----
        None

        Return:
        ------
        PoseObj: object representing how we should move

        """
        twist = Twist()
        # Turn left until we see the gate
        twist.linear.x = 0.5
        if self.yolo_contains(SemanticObject.VerticalMarker):
            quat = [self.current_odometry.orientation.x,
                    self.current_odometry.orientation.y,
                    self.current_odometry.orientation.z,
                    self.current_odometry.orientation.w]
            _, _, current_angle = tf_transformations.euler_from_quaternion(quat)
            object_angle = self.get_object_angle(SemanticObject.VerticalMarker)
            self.vertical_angle = current_angle + object_angle
            return PoseObj(cmd_twist=twist,
                           desired_depth=self.desired_depth,
                           desired_heading=self.vertical_angle,
                           movement_method=MovementMethod.VelocityMaintainDepthHeading)
        return PoseObj(cmd_twist=twist,
                       desired_depth=self.desired_depth,
                       movement_method=MovementMethod.VelocityMaintainDepth)
