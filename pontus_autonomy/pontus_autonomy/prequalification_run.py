#!/usr/bin/env python3

import rclpy
import numpy as np

from pontus_autonomy.base_run import BaseRun

# Tasks
from pontus_autonomy.tasks.gate_task import GateTask
from pontus_autonomy.tasks.vertical_marker_task import VerticalMarkerTask
from pontus_autonomy.tasks.surface_task import SurfaceTask

class PrequalificationRun(BaseRun):

    def __init__(self):
        super().__init__("prequalification_run")

        print("Starting Prequalification Run")

        # Gate Task
        result = self.run_task(GateTask)
        print("Gate Task 1: ", result)

        # Vertical Marker Task
        result = self.run_task(VerticalMarkerTask)
        print("Gate Task 1: ", result)

        # Return Gate Task
        result = self.run_task(GateTask)
        print("Gate Task 2: ", result)

        # Return to the surface
        result = self.run_task(SurfaceTask)
        print("Surfaced: ", result)


def main(args=None):
    rclpy.init(args=args)

    node = PrequalificationRun()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
