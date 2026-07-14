import rclpy
from rclpy.executors import MultiThreadedExecutor

from typing import Optional, List

from pontus_autonomy.base_run import BaseRun
from geometry_msgs.msg import PoseStamped, Point, Quaternion

# Tasks
from pontus_autonomy.tasks.localization.submerge import Submerge
from pontus_autonomy.tasks.slalom_task import SlalomTask
from pontus_autonomy.tasks.gate_task import GateTask
from pontus_autonomy.tasks.surface_octagon_task import OctagonSurfaceTask
from pontus_autonomy.tasks.return_home_replay import ReturnHomeReplayTask

from pontus_autonomy.tasks.slalom_task import SlalomSide
from pontus_autonomy.tasks.gate_task import GateSide

import numpy as np


class FullNavRun(BaseRun):

    def __init__(self):
        super().__init__("full_nav_run", 
                            handle_autonomy_switch = True,
                            handle_resetting_all_nodes = True,
                            handle_command_mode = True,
                            start_run_wait_time_s = 8
                         )
        pass

    def run_function(self):
        # ------- Run Parameters ----------
        gate_slalom_side_is_right = True
        default_depth = 0.65
        do_octagon = False

        outbound_waypoints: list[np.ndarray] = []

        # -------- Start Run -------------
        self.get_logger().info("Starting Full Navigation Run")

        # Submerge Task
        self.get_logger().info("Starting Submerge")

        submerge_task = Submerge()
        submerge_task.desired_depth = -default_depth

        result, _ = self.run_task(submerge_task)
        self.get_logger().info(f"Submerge: {result}")

        if not result:
            raise RuntimeError("Submerge task failed")

        # Gate Task
        self.get_logger().info("Starting Gate Task")

        gate_task = GateTask()
        gate_task.depth_m = default_depth
        gate_task.gate_side = (
            GateSide.RIGHT
            if gate_slalom_side_is_right
            else GateSide.LEFT
        )

        result, gate_waypoints = self.run_task(gate_task)
        self.get_logger().info(f"Gate Task: {result}")

        if not result:
            raise RuntimeError("Gate task failed")

        if gate_waypoints:
            outbound_waypoints.extend(
                waypoint.copy() for waypoint in gate_waypoints
            )

        # Slalom Task
        self.get_logger().info("Starting Slaloms")

        slalom_task = SlalomTask()
        slalom_task.depth_m = default_depth
        slalom_task.slalom_side = (
            SlalomSide.RIGHT
            if gate_slalom_side_is_right
            else SlalomSide.LEFT
        )

        result, slalom_waypoints = self.run_task(slalom_task)
        self.get_logger().info(f"Slalom Navigation Task: {result}")

        if not result:
            raise RuntimeError("Slalom task failed")

        if slalom_waypoints:
            outbound_waypoints.extend(
                waypoint.copy() for waypoint in slalom_waypoints
            )

        if do_octagon:
            self.get_logger().info("Starting Octagon Surface")

            octagon_task = OctagonSurfaceTask()
            result, octagon_waypoints = self.run_task(octagon_task)

            self.get_logger().info(f"Octagon Surface Task: {result}")

            if not result:
                raise RuntimeError("Octagon surface task failed")

            if octagon_waypoints:
                outbound_waypoints.extend(
                    waypoint.copy() for waypoint in octagon_waypoints
                )

        # Create a new list in reverse traversal order.
        return_home_path = [
            waypoint.copy()
            for waypoint in reversed(outbound_waypoints)
        ]

        # Add origin
        return_home_path.append(np.array([0.0, 0.0], dtype=float))

        self.get_logger().info(
            f"Starting Return Home with "
            f"{len(return_home_path)} waypoints"
        )

        return_home_task = ReturnHomeReplayTask()
        return_home_task.depth_m = default_depth
        return_home_task.set_path(return_home_path)

        result, _ = self.run_task(return_home_task)

        self.get_logger().info(f"Return Home Task: {result}")

# class FullNavRun(BaseRun):
#     def __init__(self):
#         super().__init__("slalom_run")

        

#         # ------- Run Parameters ----------
#         gate_slalom_side_is_right = False
#         default_depth = 0.5

#         do_octagon = False
        
#         return_home_path_preflip = []

        

#         # -------- Start Run -------------``
#         self.get_logger().info("Starting Slalom Run")

#         # Submerge Task
#         self.get_logger().info("Starting Submerge")
#         submerge_task = Submerge()
#         submerge_task.desired_depth = -default_depth
#         result, _ = self.run_task(submerge_task)
#         self.get_logger().info(f"Submerge: {result}")

#         # Gate Task Prequal
#         self.get_logger().info("Starting Gate Task")
#         gate_task = GateTask()
#         gate_task.depth_m = default_depth

#         if gate_slalom_side_is_right:
#             gate_task.gate_side = GateSide(0)
#         else:
#             gate_task.gate_side = GateSide(1)

#         result, gate_waypoints  = self.run_task(gate_task)
#         return_home_path_preflip += gate_waypoints
        
#         self.get_logger().info(f"Prequal Gate Task: {result}")

#         # Slalom Task
#         self.get_logger().info("Starting Slaloms")
#         slalom_task = SlalomTask()
#         slalom_task.depth_m = default_depth

#         if gate_slalom_side_is_right:
#             slalom_task.slalom_side = SlalomSide(0)
#         else:
#             slalom_task.slalom_side = SlalomSide(1)

#         result, slalom_waypoints = self.run_task(slalom_task)
#         return_home_path_preflip += slalom_waypoints

#         self.get_logger().info(f"Slalom Navigation Task: {result}")

#         if do_octagon:
#             # Octagon Surface Task
#             self.get_logger().info("Starting Octagon Surface")
#             result = self.run_task(OctagonSurfaceTask)
#             self.get_logger().info(f"Octagon Surface Task: {result}")

#         return_home_path = list(reversed(return_home_path_preflip))

#         # Return Home Task
#         self.get_logger().info("Starting Return Home")
#         return_home_task = ReturnHomeReplayTask()
#         return_home_task.set_path(return_home_path)

#         result, _ = self.run_task(return_home_task)

#         self.get_logger().info(f"Return Home Task: {result}")



def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = FullNavRun()

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
