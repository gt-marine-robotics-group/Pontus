import rclpy

from typing import Optional, List

from pontus_autonomy.base_run import BaseRun

# Tasks
from pontus_autonomy.tasks.localization.submerge import Submerge
from pontus_autonomy.tasks.localization.gate_task_prequal import GateTaskPrequal
from pontus_autonomy.tasks.localization.gate_to_vertical import GateToVertical
from pontus_autonomy.tasks.localization.vertical_marker_task import VerticalMarkerTask
from pontus_autonomy.tasks.localization.prequal_return import PrequalReturn


class PrequalificationRun(BaseRun):
    def __init__(self):
        super().__init__("prequalification_run")

        self.get_logger().info("Starting Prequalification Run")

        # Submerge Task
        result = self.run_task(Submerge)
        self.get_logger().info(f"Submerge: {result}")

        # Gate Task
        result = self.run_task(GateTaskPrequal)
        self.get_logger().info(f"Gate Task: {result}")

        # Navigate to next task
        result = self.run_task(GateToVertical)
        self.get_logger().info(f"Gate to vertical: {result}")

        # Vertical Marker Task
        result = self.run_task(VerticalMarkerTask)
        self.get_logger().info(f"Vertical Marker Task {result}")

        # Vertical to Gate
        result = self.run_task(PrequalReturn)
        self.get_logger().info(f"Prequal return: {result}")


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PrequalificationRun()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
