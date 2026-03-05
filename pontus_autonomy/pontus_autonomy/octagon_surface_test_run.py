import rclpy

from typing import Optional, List

from pontus_autonomy.base_run import BaseRun
from geometry_msgs.msg import PoseStamped, Point, Quaternion

# Tasks
from pontus_autonomy.tasks.localization.submerge import Submerge
from pontus_autonomy.tasks.slalom_task import SlalomTask
from pontus_autonomy.tasks.surface_octagon_task import OctagonSurfaceTask
from pontus_autonomy.tasks.prequal_gate_task import PrequalGateTask


class OctagonSurfaceRun(BaseRun):
    def __init__(self):
        super().__init__("octagon_surface_run")

        self.get_logger().info("Starting Octagon Surface Run")

        # Submerge Task
        result = self.run_task(Submerge)
        self.get_logger().info(f"Submerge: {result}")

        # Gate Task Prequal
        # result = self.run_task(PrequalGateTask)
        # self.get_logger().info(f"Prequal Gate Task: {result}")

        # Slalom Task
        result = self.run_task(OctagonSurfaceTask)
        self.get_logger().info(f"Octagon Surfacing Task: {result}")


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = OctagonSurfaceRun()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
