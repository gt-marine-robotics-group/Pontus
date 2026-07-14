import time
from pontus_autonomy.base_run import BaseTask
from std_srvs.srv import Empty
from dvl_msgs.msg import ConfigCommand

class LocalizationClient:
    def __init__(self, node: BaseTask):
        self.node = node
        self.ekf_reset_service = node.create_client(Empty, "/reset")
        self.dvl_republisher_reset_service = node.create_client(Empty, "/reset_dvl_republisher")
        self.dvl_command_pub = node.create_publisher(ConfigCommand, "/dvl/config/command", 10)

    def reset_all(self):
        self.dvl_reset_dead_reckoning()
        time.sleep(0.2)
        self.dvl_republisher_reset()
        time.sleep(0.2)
        self.ekf_reset_filter()

    def ekf_reset_filter(self):
        if self.ekf_reset_service.wait_for_service(timeout_sec=5.0):
            self.ekf_reset_service.call_async(Empty.Request())
        else:
            self.node.get_logger().warn("Failed to find EKF reset service")

    def dvl_republisher_reset(self):
        if self.dvl_republisher_reset_service.wait_for_service(timeout_sec=5.0):
            self.dvl_republisher_service.call_async(Empty.Request())
        else:
            self.node.get_logger().warn("Failed to find DVL Republisher reset service")

    def dvl_reset_dead_reckoning(self):
        cmd = ConfigCommand()
        self.command = "reset_dead_reckoning"

        self.dvl_command_pub.publish(cmd)

    def dvl_calibrate_gyro(self):
        cmd = ConfigCommand()
        self.command = "calibrate_gyro"

        self.dvl_command_pub.publish(cmd)