import math
from enum import Enum, auto
import numpy as np
from typing import Optional
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose
from image_geometry import PinholeCameraModel
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
import rclpy.client
import tf_transformations
from geometry_msgs.msg import Point

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj
from pontus_msgs.srv import GetGateLocation
from pontus_msgs.srv import GateSideInformation
from pontus_msgs.msg import SlalomDetectorResults
from pontus_controller.position_controller import MovementMethod
from pontus_autonomy.helpers.slalom_detector import SlalomPair 
from pontus_perception.color_threshold_detection import ThresholdDetection

from pontus_autonomy.helpers.slalom_geometry import heading_to_pole, distance_to_pole


# Bounding box colors
BLUE_BGR = (255, 0, 0)
GREEN_BGR = (0, 255, 0)


class SlalomState(Enum):
    SUBMERGE = auto()
    SEARCH = auto()
    NAVIGATE = auto()
    COMPLETE = auto()


class SlalomTask(BaseTask):
    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 480
    FX = 613.12164
    FOV_DEG = math.degrees(2.0 * math.atan(IMAGE_WIDTH / (2.0 * FX)))
    MARKER_DIAMETER_M = 0.0254  # 1 inch
    WAYPOINT_OFFSET_M = 1.0
    DESIRED_DEPTH = -1.2

    PAIR_IN_FRAME_MARGIN = 5

    def __init__(self):
        super().__init__("slalom_task")

        self.current_pose: Optional[Pose] = None

        self.detected_slalom_pair: Optional[SlalomPair] = None
        self.camera_model = PinholeCameraModel()
        self.camera_model.from_camera_info("/pontus/camera_2/camera_info")

        self.state = SlalomState.SUBMERGE
        self.pairs_completed = 0

        self.last_detected_point = None
        self.pair_detected = False

        self.cmd_send = False

        self.service_callback_group = MutuallyExclusiveCallbackGroup()

        self.go_to_pose_client = GoToPoseClient(self)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odometry_callback,
            10
        )

        self.slalom_detection_sub = self.create_subscription(
            SlalomDetectorResults,
            '/pontus/slalom_detector/results',
            self.slalom_results_callback,
            10
        )

        self.state_machine_timer = self.create_timer(
            0.2, self.state_machine
        )

        self.get_logger().info("Slalom Task Started")

    def odometry_callback(self, msg: Odometry) -> None:
        """
        Handle odometry callback

        Args:
        ---
        msg (Odometry): the odometry message from the topic

        Return:
        ---
        None
        """

        self.current_pose = msg.pose.pose

    def slalom_results_callback(self, msg: SlalomDetectorResults) -> None:

        if (msg.red_slalom_found):
            red_slalom: BoundingBox = BoundingBox(
                msg.red_slalom_x,
                msg.red_slalom_y,
                msg.red_slalom_w,
                msg.red_slalom_h,
                BLUE_BGR
            )
        else:
            red_slalom = None

        if (msg.white_slalom_found):
            white_slalom: BoundingBox = BoundingBox(
                msg.white_slalom_x,
                msg.white_slalom_y,
                msg.white_slalom_w,
                msg.white_slalom_h,
                GREEN_BGR
            )
        else:
            white_slalom = None

        if red_slalom and white_slalom:
            pair = SlalomPair(red_slalom, white_slalom)
            if self._pair_in_frame(pair):
                self.detected_slalom_pair = pair
            else:
                self.detected_slalom_pair = None
        else:
            self.detected_slalom_pair = None

    def _pair_in_frame(self, pair: SlalomPair) -> bool:
        margin = self.PAIR_IN_FRAME_MARGIN

        for box in pair:
            if (
                box.x <= margin
                or box.y <= margin
                or box.x + box.w >= self.IMAGE_WIDTH - margin
                or box.y + box.h >= self.IMAGE_HEIGHT - margin
            ):
                return False

        return True

    def compute_goal_pose(self, pair: SlalomPair) -> PoseObj:
        boxes = [pair.red_slalom, pair.white_slalom]
        coords = []

        for box in boxes:
            heading = heading_to_pole(
                box, self.camera_model
            )

            distance = distance_to_pole(
                box,
                self.camera_model,
                self.MARKER_DIAMETER_M
            )

            x = distance * math.cos(heading)
            y = distance * math.sin(heading)
            coords.append((x, y))

        mid_x = sum(c[0] for c in coords) / 2.0
        mid_y = sum(c[1] for c in coords) / 2.0
        norm = math.hypot(mid_x, mid_y)

        if norm > 0.0:
            mid_x += (mid_x / norm) * self.WAYPOINT_OFFSET_M
            mid_y += (mid_y / norm) * self.WAYPOINT_OFFSET_M

        q = self.current_pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )

        # TODO: Account for camera offset
        world_x = (
            self.current_pose.position.x
            + mid_x * math.cos(yaw)
            - mid_y * math.sin(yaw)
        )
        world_y = (
            self.current_pose.position.y
            + mid_x * math.sin(yaw)
            + mid_y * math.cos(yaw)
        )

        cmd_pose = Pose()
        cmd_pose.position.x = world_x
        cmd_pose.position.y = world_y
        cmd_pose.position.z = self.current_pose.position.z

        return PoseObj(
            cmd_pose=cmd_pose,
            skip_orientation=True,
            movement_method=MovementMethod.TurnThenForward,
        )

    def state_machine(self):
        pass
        
