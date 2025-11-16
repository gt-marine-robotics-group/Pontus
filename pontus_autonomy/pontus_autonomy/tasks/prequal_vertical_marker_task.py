import numpy as np
from enum import Enum 
from dataclasses import dataclass

import rclpy
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance
from nav_msgs.msg import Odometry

from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer
import tf2_geometry_msgs

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj

from pontus_msgs.msg import SemanticMap, SemanticObject


class PrequalVerticalMarkerTask(BaseTask):
    
    def __init__(self):
        super().__init__("prequal_vertical_marker_task")

        
        