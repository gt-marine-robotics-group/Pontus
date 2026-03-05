import rclpy

from typing import Optional, List

from pontus_autonomy.base_run import BaseRun
from geometry_msgs.msg import PoseStamped, Point, Quaternion

# Tasks
from pontus_autonomy.tasks.localization.submerge import Submerge
from pontus_autonomy.tasks.test_tasks.path_planning_test_task import PathPlanningTestTask


class PathPlanningRun(BaseRun):
    def __init__(self):
        super().__init__("path_planning_run")

        self.get_logger().info("Starting Path Planning Test Run")

        # Submerge Task
        result = self.run_task(Submerge)
        self.get_logger().info(f"Submerge: {result}")

        # Gate Task Prequal
        result = self.run_task(PathPlanningTestTask)
        self.get_logger().info(f"Path Planning Test: {result}")



def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PathPlanningRun()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
