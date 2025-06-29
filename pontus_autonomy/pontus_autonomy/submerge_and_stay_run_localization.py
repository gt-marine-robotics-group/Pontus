import rclpy

from typing import Optional, List

from pontus_autonomy.base_run import BaseRun

# Tasks
from pontus_autonomy.tasks.localization.submerge import Submerge

class SubmergeAndStay(BaseRun):
    def __init__(self):
        super().__init__("submerge_and_stay_run")

        self.get_logger().info("Starting Submerge and Stay")

        # Submerge Task
        result = self.run_task(Submerge)
        self.get_logger().info(f"Submerge: {result}")

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SubmergeAndStay()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()
