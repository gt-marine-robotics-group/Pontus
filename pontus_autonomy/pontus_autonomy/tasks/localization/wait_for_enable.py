from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import time
from pontus_autonomy.tasks.base_task import BaseTask

class WaitForEnable(BaseTask):
    def __init__(self):
        super().__init__('wait_for_enable')

        self.create_subscription(
            Bool,
            '/auto_enable',
            self.auto_enable,
            10
        )
        self.auto_publisher = self.create_publisher(
            Bool,
            '/autonomy_mode',
            10,
        )

    def auto_enable(self, msg):
        if msg.data:
            self.get_logger().info("Enable autonomy")
            self.complete(True)
            self.get_logger().info('Done waiting')