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

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj
from pontus_msgs.srv import GetGateLocation
from pontus_msgs.srv import GateSideInformation
from pontus_msgs.msg import YOLOResultArray
from pontus_controller.position_controller import MovementMethod