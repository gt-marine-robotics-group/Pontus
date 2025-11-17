import rclpy
from rclpy.node import Node
from pontus_autonomy.tasks.base_task import BaseTask


class BaseRun(Node):
    def __init__(self, name: str):
        super().__init__(name)

    def run_task(self, task: BaseTask, args=None) -> bool:
        """
        Execute task and return result.

        Args:
        ----
        task (BaseTask): the task we want to run

        Return:
        ------
        bool: the result of the task

        """
        if args == None:
            task = task()
        else:
            task = task(args)
        rclpy.spin_until_future_complete(task, task.wait_for_task())
        result = task.task_future.result()
        task.destroy_node()

        return result