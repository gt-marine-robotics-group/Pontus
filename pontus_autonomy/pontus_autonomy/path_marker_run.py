import rclpy


from typing import Optional, List

from pontus_autonomy.base_run import BaseRun

# Tasks
from pontus_autonomy.tasks.localization.submerge import Submerge
from pontus_autonomy.tasks.path_marker_task import PathMarkerTask

class PathMarkerRun(BaseRun):
    def __init__(self):
        super().__init__("path_marker_run")

        self.get_logger().info("Starting Path Marker Task")

        # Submerge Task
        self.get_logger().info("Submerging:")
        result = self.run_task(Submerge)
        self.get_logger().info(f"Submerge: {result}")

        # Path Marker Task 
        self.get_logger().info("Path Marker Task Starting:")
        result = self.run_task(PathMarkerTask)
        self.get_logger().info(f"Path Marker Result: {result}")


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PathMarkerRun()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()
