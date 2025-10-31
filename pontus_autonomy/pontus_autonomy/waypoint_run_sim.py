import rclpy

from typing import Optional, List

from pontus_autonomy.base_run import BaseRun
import time

# Tasks
from pontus_autonomy.tasks.localization.waypoint_controller import WaypointContoller
from pontus_autonomy.tasks.localization.wait_for_enable import WaitForEnable
import subprocess


class WaypointRun(BaseRun):
    def __init__(self):
        super().__init__("waypoint_run")

        self.get_logger().info("Starting Run")

        # result = self.run_task(WaitForEnable)
        # self.get_logger().info(f"Wait for enable: {result}")

        # process = subprocess.Popen(
        #     ['ros2', 'launch', 'pontus_bringup', 'auv.launch.py', 'auv:=auv'])
        # time.sleep(30)
        # Submerge Task
        result = self.run_task(WaypointContoller)
        self.get_logger().info(f"WaypointContoller: {result}")


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = WaypointContoller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
