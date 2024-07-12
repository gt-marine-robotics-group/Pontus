#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class BaseRun(Node):

    def __init__(self, name):
        super().__init__(name)

    def run_task(self, task):
        task = task()
        rclpy.spin_until_future_complete(task, task.wait_for_task())
        result = task.task_future.result()
        task.destroy_node()

        return result
