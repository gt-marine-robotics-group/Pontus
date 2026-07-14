import rclpy
from rclpy.executors import MultiThreadedExecutor

import numpy as np
from typing import Optional, List

from pontus_autonomy.base_run import BaseRun
from geometry_msgs.msg import PoseStamped, Point, Quaternion

# Tasks
from pontus_autonomy.tasks.localization.wait_for_enable import WaitForEnable
from pontus_autonomy.tasks.localization.submerge import Submerge
from pontus_autonomy.tasks.prequal_gate_task import PrequalGateTask
from pontus_autonomy.tasks.prequal_vertical_marker_task import PrequalVerticalMarkerTask

# from pontus_autonomy.tasks.table_search_task import TableSearchTask

from pontus_autonomy.tasks.prequal_search_gate_task import PrequalSearchTask
# from pontus_autonomy.tasks.search_task import ScanTask, SearchConditions

from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj

import math
import time
import subprocess


class PrequalificationRun(BaseRun):
    def __init__(self):
        super().__init__(
            "prequalification_run_sim",
            handle_autonomy_switch=True
            )
        pass

    def run_function(self):

        # Submerge Task
        self.get_logger().info("Starging Submerge")
        result = self.run_task(Submerge())
        self.get_logger().info(f"Submerge: {result}")

        # self.get_logger().info(f"Prequal Gate Searching")
        # result = self.run_task(ScanTask, (-np.pi/3, np.pi/3, 0))
        # self.get_logger().info(f"Search: {result}")

        # if (not result):
        #     self.get_logger().info("Ending run")
        #     return

        # self.get_logger().info("Starting Gate Execution")
        # result = self.run_task(PrequalGateTask)
        # self.get_logger().info(f"Prequal Gate Task: {result}")

        return True

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PrequalificationRun()

    run_executor = MultiThreadedExecutor()   
    run_executor.add_node(node)

    try:
        run_executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        run_executor.shutdown()
        rclpy.shutdown()
