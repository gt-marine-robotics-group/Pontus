import rclpy

from typing import Optional, List

from pontus_autonomy.base_run import BaseRun
from geometry_msgs.msg import PoseStamped, Point, Quaternion

# Tasks
from pontus_autonomy.tasks.localization.submerge import Submerge
from pontus_autonomy.tasks.prequal_gate_task import PrequalGateTask
from pontus_autonomy.tasks.prequal_vertical_marker_task import PrequalVerticalMarkerTask

# from pontus_autonomy.tasks.table_search_task import TableSearchTask

from pontus_autonomy.tasks.prequal_search_gate_task import PrequalSearchTask
from pontus_autonomy.tasks.search_task import ScanTask, SearchConditions

import math


class PrequalificationRun(BaseRun):
    def __init__(self):
        super().__init__("prequalification_run_sim")

        self.get_logger().info("Starting Prequalification Run")

        # Submerge Task

        self.get_logger().info("Starging Submerge")
        result = self.run_task(Submerge)
        self.get_logger().info(f"Submerge: {result}")

        self.get_logger().info("Starging Gate Scan")
        result = self.run_task(ScanTask,
                               terminating_condition=SearchConditions.GATE)
        self.get_logger().info(f"Prequal Gate Task: {result}")

        # Gate Task Prequal
        self.get_logger().info("Starting Gate Execution")
        result = self.run_task(PrequalGateTask)
        self.get_logger().info(f"Prequal Gate Task: {result}")

        # Marker Task Prequal
        self.get_logger().info("Starting Marker Scan")
        result = self.run_task(ScanTask,
                               target_angle1_rad=math.radians(15),
                               target_angle2_rad=math.radians(-15),
                               terminating_condition=SearchConditions.MARKER)
        self.get_logger().info(f"Prequal Vertical Marker Search: {result}")

        self.get_logger().info("Starting Vertical Marker Execution")
        result = self.run_task(PrequalVerticalMarkerTask,
                               fallback_distance=2.2)
        self.get_logger().info(f"Prequal Vertical Marker Task: {result}")


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PrequalificationRun()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
