import rclpy

from typing import Optional, List

from pontus_autonomy.base_run import BaseRun
from geometry_msgs.msg import PoseStamped, Point, Quaternion

# Tasks
from pontus_autonomy.tasks.localization.submerge import Submerge
from pontus_autonomy.tasks.slalom_task import SlalomTask
from pontus_autonomy.tasks.gate_task import GateTask
from pontus_autonomy.tasks.surface_octagon_task import OctagonSurfaceTask
from pontus_autonomy.tasks.return_home import ReturnHomeTask


class FullNavRun(BaseRun):
    def __init__(self):
        super().__init__("slalom_run")

        self.get_logger().info("Starting Slalom Run")

        # Submerge Task
        self.get_logger().info("Starting Submerge")
        result = self.run_task(Submerge)
        self.get_logger().info(f"Submerge: {result}")

        # Gate Task Prequal
        self.get_logger().info("Starting Gate Task")
        result = self.run_task(GateTask)
        self.get_logger().info(f"Prequal Gate Task: {result}")

        # Slalom Task
        self.get_logger().info("Starting Slaloms")
        result = self.run_task(SlalomTask)
        self.get_logger().info(f"Slalom Navigation Task: {result}")

        # Octagon Surface Task
        self.get_logger().info("Starting Octagon Surface")
        result = self.run_task(OctagonSurfaceTask)
        self.get_logger().info(f"Octagon Surface Task: {result}")

        # Return Home Task
        self.get_logger().info("Starting Return Home")
        result = self.run_task(ReturnHomeTask)
        self.get_logger().info(f"Return Home Task: {result}")



def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = FullNavRun()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
