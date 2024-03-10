import rclpy

from typing import Optional, List

from pontus_autonomy.base_run import BaseRun

# Tasks
from pontus_autonomy.tasks.localization.submerge import Submerge
from pontus_autonomy.tasks.localization.gate_task import GateTask


class SemiRun(BaseRun):
    def __init__(self):
        super().__init__("semi_run")

        self.get_logger().info("Starting semi Run")

        # Submerge Task
        result = self.run_task(Submerge)
        self.get_logger().info(f"Submerge: {result}")

        # Gate Task
        result = self.run_task(GateTask)
        self.get_logger().info(f"Gate Task: {result}")


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SemiRun()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()
