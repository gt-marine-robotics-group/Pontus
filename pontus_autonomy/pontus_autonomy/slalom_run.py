import rclpy

from std_srvs.srv import SetBool
from typing import Optional, List

from pontus_autonomy.base_run import BaseRun

# Tasks
from pontus_autonomy.tasks.localization.submerge import Submerge
from pontus_autonomy.tasks.slalom_task import SlalomTask

class SlalomRun(BaseRun):
    def __init__(self):
        super().__init__("slalom_run")

        self.slalom_detection_set_enable_client = self.create_client(
            SetBool,
            '/pontus/slalom_detector/set_enabled',
        )

        self.get_logger().info("Starting Slalom Task")

        # Submerge Task
        self.get_logger().info("Submerging:")
        result = self.run_task(Submerge)
        self.get_logger().info(f"Submerge: {result}")

        # SLalom Task 
        self.get_logger().info("Slalom Task Starting:")
        self.send_slalom_detector_enable_request(True)
        result = self.run_task(SlalomTask)
        self.get_logger().info(f"Slalom Result: {result}")


    def send_slalom_detector_enable_request(self, enable: bool) -> None:
        req = SetBool.Request()
        req.data = enable

        future = self.slalom_detection_set_enable_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(
                f"success={future.result().success} "
                f"msg='{future.result().message}'")
        else:
            self.get_logger().error("Slalom Set Enable Service call failed")

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SlalomRun()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()
