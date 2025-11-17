import rclpy

from typing import Optional, List

from pontus_autonomy.base_run import BaseRun
from geometry_msgs.msg import PoseStamped, Point, Quaternion

# Tasks
from pontus_autonomy.tasks.localization.submerge import Submerge
from pontus_autonomy.tasks.prequal_gate_task import PrequalGateTask
from pontus_autonomy.tasks.prequal_vertical_marker_task import PrequalVerticalMarkerTask

from pontus_autonomy.tasks.prequal_search_gate_task import PrequalSearchTask

class PrequalificationRun(BaseRun):
    def __init__(self):
        super().__init__("prequalification_run_sim")

        self.get_logger().info("Starting Prequalification Run")

        # Submerge Task
        #result = self.run_task(Submerge)
        #self.get_logger().info(f"Submerge: {result}")
        fallback_points = self._define_fallback_points()
        result = self.run_task(PrequalSearchTask, fallback_points)
        self.get_logger().info(f"Prequal Gate Task: {result}")

        # Gate Task Prequal
        result = self.run_task(PrequalGateTask)
        self.get_logger().info(f"Prequal Gate Task: {result}")

        # Marker Task Prequal
        result = self.run_task(PrequalVerticalMarkerTask)
        self.get_logger().info(f"Prequal Vertical Marker Task: {result}")

    def _define_fallback_points(self) -> list:
        """
        Defines the fallback points for Prequal Search Task should it fail to find the gate
        
        Currently, the current fallback value are arbitrary and should be defined
        """
        fallback_points = [[2, PoseStamped()], [3, PoseStamped()]]
        
        
        fallback_points[0][1].header.frame_id = "map"
        fallback_points[0][1].pose.position = Point(
            x=float(1),
            y=float(2),
            z=float(2)
        )
        
        fallback_points[0][1].pose.orientation = Quaternion(
            x=float(0),
            y=float(0),
            z=float(0),
            w=float(0)
        )
        
        fallback_points[1][1].header.frame_id = "map"
        fallback_points[1][1].pose.position = Point(
            x=float(2),
            y=float(3),
            z=float(2)
        )
        
        fallback_points[1][1].pose.orientation = Quaternion(
            x=float(0),
            y=float(0),
            z=float(0),
            w=float(0)
        )
        
        return fallback_points

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PrequalificationRun()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
