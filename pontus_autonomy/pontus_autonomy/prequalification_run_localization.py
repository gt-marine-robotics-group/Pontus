import rclpy

from typing import Optional, List

from pontus_autonomy.base_run import BaseRun

# Tasks
from pontus_autonomy.tasks.localization.submerge import Submerge
from pontus_autonomy.tasks.localization.gate_task_prequal_velocity import GateTaskPrequalVelocity
from pontus_autonomy.tasks.localization.vertical_marker_task_velocity import VerticalMarkerTaskVelocity  # noqa E501
from pontus_autonomy.tasks.localization.prequal_return import PrequalReturn


class PrequalificationRun(BaseRun):
    def __init__(self):
        super().__init__("prequalification_run")

        self.get_logger().info("Starting Prequalification Run")

        # Submerge Task
        result = self.run_task(Submerge)
        self.get_logger().info(f"Submerge: {result}")

        # Gate Task
        result = self.run_task(GateTaskPrequalVelocity)
        self.get_logger().info(f"Gate Task: {result}")

        # Vertical Marker
        result = self.run_task(VerticalMarkerTaskVelocity)
        self.get_logger().info(f"Vertical Marker Task: {result}")

        # Pass back through gate
        result = self.run_task(PrequalReturn)
        self.get_logger().info(f"Prequal Return Task: {result}")


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PrequalificationRun()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
