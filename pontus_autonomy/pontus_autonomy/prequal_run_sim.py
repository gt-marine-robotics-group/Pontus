import rclpy

from typing import Optional, List

from pontus_autonomy.base_run import BaseRun

# Tasks
from pontus_autonomy.tasks.localization.submerge import Submerge
from pontus_autonomy.tasks.prequal_gate_task import PrequalGateTask
from pontus_autonomy.tasks.prequal_vertical_marker_task import PrequalVerticalMarkerTask


class PrequalificationRun(BaseRun):
    def __init__(self):
        super().__init__("prequalification_run_sim")

        self.get_logger().info("Starting Prequalification Run")

        # Submerge Task
        result = self.run_task(Submerge)
        self.get_logger().info(f"Submerge: {result}")

        # Gate Task Prequal
        result = self.run_task(PrequalGateTask)
        self.get_logger().info(f"Prequal Gate Task: {result}")

        # Marker Task Prequal
        result = self.run_task(PrequalVerticalMarkerTask)
        self.get_logger().info(f"Prequal Vertical Marker Task: {result}")


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PrequalificationRun()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
