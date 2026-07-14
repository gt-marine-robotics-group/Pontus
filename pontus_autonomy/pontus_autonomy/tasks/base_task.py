import rclpy
from rclpy.node import Node
import time

class BaseTask(Node):
    debug_string = ""
    overlay_image = None

    def __init__(self, name: str):
        seconds = round(time.time())
        self.name = f"{name}_{str(seconds)[-4:]}"
        super().__init__(self.name)

        self.task_future = rclpy.task.Future()

    def wait_for_task(self) -> None:
        """
        Return task future.

        Args:
        ----
        None

        Return:
        ------
        None

        """
        return self.task_future

    def complete(self, success: bool) -> None:
        """
        Set the result of the future.

        Args:
        ----
        success (bool): whether or not the task succeeded

        Return:
        ------
        None

        """
        self.task_future.set_result(success)
