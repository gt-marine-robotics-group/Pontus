from pontus_autonomy.base_run import BaseTask
from dvl_msgs.msg import ConfigCommand

class GoToPoseClient:
    def __init__(self, node: BaseTask):
        self.dvl_command_pub = node.create_publisher(ConfigCommand, "dvl/config/command", 10)

    def reset_dead_reckoning(self):
        cmd = ConfigCommand()
        self.command = "reset_dead_reckoning"

        self.dvl_command_pub.publish(cmd)

    def calibrate_gyro(self):
        cmd = ConfigCommand()
        self.command = "calibrate_gyro"

        self.dvl_command_pub.publish(cmd)