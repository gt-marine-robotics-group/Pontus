import rclpy
from rclpy.node import Node


class BaseTask(Node):
    debug_string = ""
    overlay_image = None

    def __init__(self, name: str):
        super().__init__(name)

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
