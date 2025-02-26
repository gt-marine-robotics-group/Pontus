from enum import Enum
import numpy as np

import rclpy

from pontus_autonomy.tasks.base_task import BaseTask

class GateState(Enum):
    Searching = 0

class GateTask(BaseTask):
    def __init__(self):
        super().__init__('gate_task')

