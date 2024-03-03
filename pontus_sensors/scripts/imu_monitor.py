#!/usr/bin/env python3

# This file basically reads from the topic /microstrain_inertial_driver/transition_event to see whether the sensor fails to initiate, if it does, retry launching the file

import rclpy
from rclpy.node import Node
from lifecycle_msgs.msg import TransitionEvent
from sensor_msgs.msg import Imu
import subprocess
from threading import Timer
import time
import os 

class ImuMonitorNode(Node):
    def __init__(self):
        super().__init__('imu_monitor_node')

        self.latest_transition_event = None
        self.timer = None
        self.subprocess_handle = None
        self.redundant_timer = None
        self.imu_data = None

        self.subscription = self.create_subscription(
            TransitionEvent,
            '/microstrain_inertial_driver/transition_event',
            self.state_callback,
            10
        )

        self.get_logger().info('First IMU launch attempt...')
        self.launch_imu_node()
        self.get_logger().info('Launched IMU...')

        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
    def __del__(self):
        self.redundant_timer.cancel()
        self.timer.cancel()
        self.subprocess_handle.terminate()
        self.subprocess_handle.wait()
        if self.subprocess_handle.poll() is None:
            self.get_logger().info("Forcibly terminating the process...")
            self.subprocess_handle.kill()
        os.system("killall microstrain_inertial_driver")

    def imu_callback(self, msg):
        self.imu_data = msg
        self.redundant_timer.cancel()
        self.redundant_timer = Timer(10.0, self.retry_launch)
        self.redundant_timer.start()

    def check_no_data_IMU(self):
        if self.imu_data == None:
            self.get_logger().info('Backup restart')
            self.retry_launch() 


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
        else:
            self.timer.cancel()
        
    def launch_imu_node(self):
        if self.redundant_timer:
            self.redundant_timer.cancel()
        self.redundant_timer = Timer(7.0, self.check_no_data_IMU)
        self.redundant_timer.start()
        if self.subprocess_handle:
            # Terminate the existing subprocess if it exists
            self.get_logger().info("Stopping")
            self.subprocess_handle.terminate()
            self.subprocess_handle.wait()

            if self.subprocess_handle.poll() is None:
                self.get_logger().info("Forcibly terminating the process...")
                self.subprocess_handle.kill()
        
        os.system("killall microstrain_inertial_driver")
        launch_cmd = ['ros2', 'launch', 'pontus_sensors', 'imu.launch.py']
        self.subprocess_handle = subprocess.Popen(launch_cmd)  # Start the launch process asynchronously

    def retry_launch(self):
        self.get_logger().info("Retrying to launch sensor driver...")
        self.launch_imu_node()

def main(args=None):
    rclpy.init(args=args)
    imu_monitor_node = ImuMonitorNode()
    try:
        rclpy.spin(imu_monitor_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_monitor_node.destroy_node()

if __name__ == '__main__':
    main()

