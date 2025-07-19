from enum import Enum
import numpy as np
from typing import Optional
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose
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
from pontus_perception.slalom_detector import SlalomPair, BoundingBox


class SlalomTask(BaseTask):

    def __init__(self):
        super().__init__("slalom_task")

        self.current_pose = None

        self.detected_slalom_pair = None
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

        self.timer = self.create_timer(
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
            red_slalom : BoundingBox = BoundingBox(
                msg.red_slalom_x,
                msg.red_slalom_y,
                msg.red_slalom_w,
                msg.red_slalom_h
            )
        else:
            red_slalom = None

        if (msg.white_slalom_found):
            white_slalom : BoundingBox = BoundingBox(
                msg.white_slalom_x,
                msg.white_slalom_y,
                msg.white_slalom_w,
                msg.white_slalom_h
            )
        else:
            white_slalom = None

        pair  = SlalomPair(red_slalom, white_slalom) 

        if red_slalom or white_slalom and self.last_detected_point:
            self.pair_detected = True

        if red_slalom and white_slalom:
            self.detected_slalom_pair = pair
            self.last_detected_point = pair.midpoint()

    def state_machine(self):
        pass
