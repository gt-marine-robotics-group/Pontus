#!/usr/bin/env python3

import rclpy
import numpy as np

from pontus_autonomy.base_run import BaseRun

# Tasks
from pontus_autonomy.tasks.localization.gate_task import GateTask
from pontus_autonomy.tasks.localization.gate_to_vertical import GateToVertical
from pontus_autonomy.tasks.localization.vertical_marker_task import VerticalMarkerTask
from pontus_autonomy.tasks.surface_task import SurfaceTask

class PrequalificationRun(BaseRun):

    def __init__(self):
        super().__init__("prequalification_run")

        self.get_logger().info("Starting Prequalification Run")

        # Gate Task
        result = self.run_task(GateTask)
        self.get_logger().info(f"Gate Task: {result}")

        # Navigate to next task
        result = self.run_task(GateToVertical)
        self.get_logger().info(f"Gate to vertical: {result}")

        # Vertical Marker Task
        result = self.run_task(VerticalMarkerTask)
        self.get_logger().info(f"Vertical Marker Task {result}")

        # Return Gate Task
        result = self.run_task(GateTask)
        self.get_logger().info(f"Return gate task: {result}")

        # Return to the surface
        result = self.run_task(SurfaceTask)
        self.get_logger().info(f"Surfaced: {result}")


def main(args=None):
    rclpy.init(args=args)

    node = PrequalificationRun()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
