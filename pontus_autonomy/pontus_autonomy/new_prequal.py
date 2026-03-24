
import rclpy

from typing import Optional, List

from pontus_autonomy.base_run import BaseRun
from geometry_msgs.msg import PoseStamped, Point, Quaternion

# Tasks
from pontus_autonomy.tasks.localization.submerge import Submerge
from pontus_autonomy.tasks.prequal_gate_task import PrequalGateTask
from pontus_autonomy.tasks.prequal_vertical_marker_task import PrequalVerticalMarkerTask

from pontus_autonomy.tasks.prequal_search_gate_task import PrequalSearchTask

class PrequalificationRun(BaseRun):
    def __init__(self):
        super().__init__("new_prequalification_run_sim")
        
        self.get_logger().info("Starting Prequalification Run")

        # Submerge Task
        result = self.run_task(Submerge)
        self.get_logger().info(f"Submerge: {result}")
        
        # Gate Prequal Task