#!/usr/bin/env python3

# This file basically reads from the topic /microstrain_inertial_driver/transition_event to see whether the sensor fails to initiate, if it does, retry launching the file

import rclpy
from rclpy.node import Node
from lifecycle_msgs.msg import TransitionEvent
import subprocess
from threading import Timer

class ImuMonitorNode(Node):
    def __init__(self):
        super().__init__('imu_monitor_node')

        self.latest_transition_event = None
        self.timer = None
        self.subprocess_handle = None

        self.subscription = self.create_subscription(
            TransitionEvent,
            '/microstrain_inertial_driver/transition_event',
            self.state_callback,
            10
        )

        self.get_logger().info('First IMU launch attempt...')
        self.launch_imu_node()
        self.get_logger().info('Launched IMU...')

    def launch_imu_node(self):
        if self.subprocess_handle:
            # Terminate the existing subprocess if it exists
            self.subprocess_handle.terminate()
            self.subprocess_handle.wait()
        launch_cmd = ['ros2', 'launch', 'pontus_sensors', 'imu.launch.py']
        self.subprocess_handle = subprocess.Popen(launch_cmd)  # Start the launch process asynchronously

    def state_callback(self, msg : TransitionEvent):
        self.get_logger().info('Received transition event...')
        self.latest_transition_event = msg
        if self.timer:
            self.timer.cancel()
        self.timer = Timer(4.0, self.check_last_transition_event)  # Set the timer for 4 seconds
        self.timer.start()

    def check_last_transition_event(self):
        # Check if the latest transition event exists and has a goal state of "active"
        self.get_logger().info('Checking last transition event...')
        if self.latest_transition_event.goal_state.label != "active":
            self.retry_launch()

    def retry_launch(self):
        self.get_logger().info("Retrying to launch sensor driver...")
        self.launch_imu_node()

def main(args=None):
    rclpy.init(args=args)
    imu_monitor_node = ImuMonitorNode()
    rclpy.spin(imu_monitor_node)
    imu_monitor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

