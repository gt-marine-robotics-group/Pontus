from std_msgs.msg import Bool
from pontus_autonomy.tasks.base_task import BaseTask

DEBOUNCE_THRESHOLD = 20


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

        self.debounce_counter = 0

    def auto_enable(self, msg):
        if msg.data:
            self.debounce_counter += 1
        else:
            self.debounce_counter = 0

        if self.debounce_counter >= DEBOUNCE_THRESHOLD:
            self.get_logger().info("Enable autonomy")
            self.complete(True)
            self.get_logger().info('Done waiting')
