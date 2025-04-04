from enum import Enum
import numpy as np

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
import tf_transformations

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj
from pontus_controller.position_controller import MovementMethod
from pontus_msgs.msg import YOLOResultArray, YOLOResultArrayPose
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

        self.create_subscription(
            YOLOResultArrayPose,
            '/pontus/camera_2/yolo_results_pose',
            self.yolo_results_pose_callback,
            10
        )

        self.current_detections = None
        self.current_detections_pose = None

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
        self.previous_state = None
        self.starting_pose = None
        self.command_sent = False
        self.current_desired_position = 0

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

    def yolo_results_pose_callback(self, msg: YOLOResultArrayPose) -> None:
        """
        Handle yolo results pose callback.

        Args:
        ----
        msg (YOLOResultArrayPose): the yolo result array pose

        Return:
        ------
        None

        """
        self.current_detections_pose = msg

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

    def yolo_pose_contains(self, obj: SemanticObject) -> bool:
        """
        Return if there exists the obj in the yolo detection.

        Args:
        ----
        obj (SemanticObject): the object we are looking for

        Return:
        ------
        bool: if we see the object or not

        """
        for detection, _ in zip(self.current_detections_pose.results,
                                self.current_detections_pose.distances):
            if detection.class_id == obj.value:
                return True
        return False

    def get_object_pose(self, obj: SemanticObject) -> Pose:
        """
        Return the pose of an object from the yolo detection.

        Args:
        ----
        obj (SemanticObject): the object we want to find

        Return:
        ------
        Pose: the pose of the object

        """
        for detection, pose in zip(self.current_detections_pose.results,
                                   self.current_detections_pose.distances):
            if detection.class_id == obj.value:
                return pose
        return None

    def copy_pose(self, pose: Pose) -> Pose:
        """
        Deep copies the given pose.

        Args:
        ----
        pose (Pose): the pose we want to copy

        Return:
        ------
        Pose: the copied pose

        """
        new_pose = Pose()
        new_pose.position.x = pose.position.x
        new_pose.position.y = pose.position.y
        new_pose.position.z = pose.position.z
        new_pose.orientation.x = pose.orientation.x
        new_pose.orientation.y = pose.orientation.y
        new_pose.orientation.z = pose.orientation.z
        new_pose.orientation.w = pose.orientation.w
        return new_pose

    def transform_body_to_odom(self, x: float, y: float, yaw: float) -> tuple[float, float]:
        """
        Rotate a point in body frame into odom frame.

        Args:
        ----
        x (float): x coordinate of point
        y (float): y coordinate of point
        yaw (float): current yaw

        Return:
        ------
        tuple[float, float]: x and y in body frame

        """
        new_x = x * np.cos(yaw) - y * np.sin(yaw)
        new_y = x * np.sin(yaw) + y * np.cos(yaw)
        return new_x, new_y

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

        if self.current_detections_pose is None:
            self.get_logger().warn("Have not received yolo detections pose yet, skipping")
            return
        self.state_debugger()
        pose_obj = None
        match self.state:
            case self.State.VerticalApproach:
                pose_obj = self.approach_vertical()
            case self.State.VerticalCircumnavigate:
                pose_obj = self.circumnavigate_vertical()
            case self.State.VerticalDone:
                self.complete(True)
            case _:
                pass

        if pose_obj is not None:
            self.go_to_pose_client.go_to_pose(pose_obj)

    def approach_vertical(self) -> PoseObj:
        """
        Approach the vertical marker.

        This will keep on going forward till we see a certain width of the gate. Then
        we will move on to the circumnavigate task.

        Args:
        ----
        None

        Return:
        ------
        PoseObj: object representing how we should move

        """
        twist = Twist()
        if self.yolo_pose_contains(SemanticObject.VerticalMarker):
            pose = self.get_object_pose(SemanticObject.VerticalMarker)
            self.get_logger().info(f"{pose.position.x}")
            if pose.position.x < 1.0 and pose.position.x != -1.0:
                self.state = self.State.VerticalCircumnavigate
                self.starting_pose = self.current_odometry
                return PoseObj(cmd_twist=twist,
                               desired_depth=self.desired_depth,
                               movement_method=MovementMethod.VelocityMaintainDepth)
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

    def circumnavigate_vertical(self) -> PoseObj:
        """
        Circumnavigate around the vertical marker.

        Args:
        ----
        None

        Return:
        ------
        PoseObj: object representing how we should move

        """
        cmd_pose = Pose()
        quat = [self.starting_pose.orientation.x,
                self.starting_pose.orientation.y,
                self.starting_pose.orientation.z,
                self.starting_pose.orientation.w]
        _, _, starting_yaw = tf_transformations.euler_from_quaternion(quat)
        # First pose
        first_pose = self.copy_pose(self.starting_pose)
        new_x, new_y = self.transform_body_to_odom(0.0, 0.9, starting_yaw)
        first_pose.position.x += new_x
        first_pose.position.y += new_y

        # Second pose
        second_pose = self.copy_pose(self.starting_pose)
        new_x, new_y = self.transform_body_to_odom(2.3, 0.9, starting_yaw)
        second_pose.position.x += new_x
        second_pose.position.y += new_y

        # Third Pose
        third_pose = self.copy_pose(self.starting_pose)
        new_x, new_y = self.transform_body_to_odom(2.3, -0.9, starting_yaw)
        third_pose.position.x += new_x
        third_pose.position.y += new_y

        # Fourth Pose
        fourth_pose = self.copy_pose(self.starting_pose)
        new_x, new_y = self.transform_body_to_odom(0.0, -0.9, starting_yaw)

        fourth_pose.position.x += new_x
        fourth_pose.position.y += new_y

        desired_positions = [first_pose, second_pose, third_pose, fourth_pose, self.starting_pose]

        if not self.command_sent:
            cmd_pose = desired_positions[self.current_desired_position]
            self.command_sent = True
            if self.current_desired_position == 1 or self.current_desired_position == 3:
                return PoseObj(cmd_pose=cmd_pose,
                               skip_orientation=True,
                               movement_method=MovementMethod.TurnThenForward)
            else:
                return PoseObj(cmd_pose=cmd_pose,
                               skip_orientation=True,
                               movement_method=MovementMethod.StrafeThenForward)

        elif self.go_to_pose_client.at_pose():
            self.current_desired_position += 1
            self.command_sent = False

        if self.current_desired_position == len(desired_positions):
            self.state = self.State.VerticalDone

        return None
