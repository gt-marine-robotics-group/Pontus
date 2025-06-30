import rclpy

from typing import Optional, List

from pontus_autonomy.base_run import BaseRun
import time

# Tasks
from pontus_autonomy.tasks.localization.submerge import Submerge
from pontus_autonomy.tasks.localization.gate_task_prequal_velocity import GateTaskPrequalVelocity
from pontus_autonomy.tasks.localization.vertical_marker_task_velocity import VerticalMarkerTaskVelocity  # noqa E501
from pontus_autonomy.tasks.localization.waypoint_controller import WaypointContoller
from pontus_autonomy.tasks.localization.wait_for_enable import WaitForEnable
import subprocess

class PrequalificationRun(BaseRun):
    def __init__(self):
        super().__init__("prequalification_run")

        self.get_logger().info("Starting Prequalification Run")

        result = self.run_task(WaitForEnable)
        self.get_logger().info(f"Wait for enable: {result}")

        process = subprocess.Popen(['ros2', 'launch', 'pontus_bringup', 'auv.launch.py', 'auv:=auv'])
        time.sleep(5)
        # Submerge Task
        result = self.run_task(WaypointContoller)
        self.get_logger().info(f"WaypointContoller: {result}")


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PrequalificationRun()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
