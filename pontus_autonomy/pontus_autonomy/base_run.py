import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.context import Context
from pontus_autonomy.tasks.base_task import BaseTask
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool
from pontus_msgs.msg import CommandMode

from pontus_autonomy.helpers.LocalizationClient import LocalizationClient
from pontus_autonomy.helpers.MapClient import MapClient

AUTONOMY_SWITCH_DEBOUNCE_THRESHOLD = 20

class BaseRun(Node):
    def __init__(self,
                 name: str, 
                 handle_autonomy_switch: bool = False,
                 handle_resetting_all_nodes: bool = True,
                 handle_command_mode: bool = True,
                 start_run_wait_time_s: int = 8):
        super().__init__(name)
        self.waypoints = []

        self.get_logger().info(f"Starting {name}")

        self.handle_autonomy_switch = handle_autonomy_switch
        self.handle_resetting_all_nodes = handle_resetting_all_nodes
        self.handle_command_mode = handle_command_mode
        self.start_run_wait_time_s = start_run_wait_time_s

        self.run_callback_group = MutuallyExclusiveCallbackGroup()

        self.command_pub = self.create_publisher(CommandMode, "/command_mode", 10)
        self.localization_client = LocalizationClient(self)
        self.map_client = MapClient(self)

        self.current_task = None
        self.running = False

        # Allows the autonomy switch callback to externally trigger the run
        self.run_trigger = self.create_timer(0.1, self.start_run)

        if self.handle_autonomy_switch:
            self.autonomy_switch_sub = self.create_subscription(
                Bool,
                '/pontus/autonomy_switch',
                self.autonomy_switch_callback,
                1,
                callback_group=self.run_callback_group
            )
            self.autonomy_switch_state = False
            self.autonomy_switch_debounce_counter = 0
            self.get_logger().info("Waiting for Autonomy Switch")

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
        self.get_logger().info(f"RUNNING TASK: {task.name}")
        self.current_task = task
        future = self.current_task.wait_for_task()

        while not future.done():
            if not self.running:
                has_waypoints = hasattr(self.current_task, "waypoints")
                self.cleanup_task()
                if has_waypoints:
                    return None, None
                return None

            rclpy.spin_once(self.current_task)

        result = self.current_task.task_future.result()

        waypoints = None
        if hasattr(self.current_task, "waypoints"):
            waypoints = self.current_task.waypoints.copy()

        self.cleanup_task()

        if waypoints is not None:
            return result, waypoints
        return result

    def run_function(self):
        self.get_logger().warn("NO RUN FUNCTION PROVIDED TO RUN CLASS")

    def reset_all(self):
        self.localization_client.reset_all()
        self.map_client.reset_map()
        time.sleep(2) # Give time for localization to settle

    def set_command_mode(self, mode):
        cmd_mode = CommandMode()
        cmd_mode.command_mode = mode
        for i in range(10):
            self.command_pub.publish(cmd_mode)
            time.sleep(0.02)

    def start_run(self):
        # if shouldn't be running or we are already running tasks
        if not self.running or self.current_task != None:
            return

        if self.handle_resetting_all_nodes:
            self.get_logger().info("Resetting system nodes")
            self.reset_all()

        self.get_logger().info("Starting run, setting CommandMode to position with strafe")

        if self.handle_command_mode:
            self.set_command_mode(CommandMode.POSITION_WITH_STRAFE)

        if callable(self.run_function):
            result = self.run_function()
            if result:
                self.get_logger().info("RUN SUCCESS!")
            self.stop_run()

    def stop_run(self):
        self.get_logger().info("Stopping Run, setting Software Estop")

        self.running = False
        self.cleanup_task()

        if self.handle_command_mode:
            self.set_command_mode(CommandMode.ESTOP)

    def autonomy_switch_callback(self, msg):
        if msg.data != self.autonomy_switch_state:
            self.autonomy_switch_debounce_counter += 1
        else:
            self.autonomy_switch_debounce_counter = 0

        if self.autonomy_switch_debounce_counter >= AUTONOMY_SWITCH_DEBOUNCE_THRESHOLD:
            self.autonomy_switch_state = msg.data

            if (self.autonomy_switch_state):
                self.get_logger().info("Autonomy Switch Enabled")

                # Give time for people to get clear of sub
                for i in range(self.start_run_wait_time_s, 0, -1):
                    print(f"\r\033[K\033[31mSafety Delay: {i}s\033[0m", end="", flush=True)
                    time.sleep(1)

                if i > 0:
                    print(f"\r\033[K\033[32mSafety Delay Complete\033[0m", flush=True)

                self.get_logger().info("Running:")
                self.running = True
            else:
                self.get_logger().info("Autonomy switch Disabled:")
                self.stop_run()

    def cleanup_task(self):
        if self.current_task is not None:
            self.current_task.destroy_node()
