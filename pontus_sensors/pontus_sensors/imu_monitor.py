#!/usr/bin/env python3

# This file basically reads from the topic /microstrain_inertial_driver/transition_event to see whether the sensor fails to initiate, if it does, retry launching the file

import rclpy
from rclpy.node import Node
from lifecycle_msgs.msg import TransitionEvent
from sensor_msgs.msg import Imu
import subprocess
from threading import Timer
import os 
import serial
import yaml
import time
import ament_index_python

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

        # Get Serial Port
        _PONTUS_IMU_PARAMS_FILE = os.path.join(
        ament_index_python.packages.get_package_share_directory('pontus_sensors'),
        'config',
        'microstrain.yaml'
        )

        # Read the YAML file
        with open(_PONTUS_IMU_PARAMS_FILE, 'r') as file:
            yaml_data = yaml.safe_load(file)

        # Extract the port parameter
        self.port = yaml_data['microstrain_inertial_driver']['ros__parameters']['port']
        # Attempt to launch the node

        self.get_logger().info('First IMU launch attempt...')
        self.launch_imu_node()
        self.get_logger().info('Launched IMU...')

        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
    def power_cycle_microstrain_driver(self):
        try:
            self.get_logger().info("Trying to power cycle imu")
            ser = serial.Serial(self.port, 9600, timeout=1)  # Adjust the port and baud rate as needed
            ser.write(b"POWER_OFF\r\n")  # Send command to power off
            time.sleep(2)  # Wait for the device to power off
            ser.write(b"POWER_ON\r\n")   # Send command to power on
            ser.close()
        except serial.SerialException as e:
            self.get_logger().info("Failed to power cycle imu")

    # This is in charge of retrying the launch if we have not gotten any data from the IMU after 10 seconds.
    def imu_callback(self, msg):
        self.imu_data = msg
        self.redundant_timer.cancel()
        # after 10 seconds retry the launch
        self.redundant_timer = Timer(10.0, self.retry_launch)
        self.redundant_timer.start()

    # If there is no data coming from the IMU, restart the launch file
    def check_no_data_IMU(self):
        if self.imu_data == None:
            self.get_logger().info('Backup restart')
            self.retry_launch() 

    # This updates the last transition event. 
    # If after 4 seconds there are no more transition updates, we assume that the sensor has transitioned into its final state.
    # Check if the last transition is activate
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

    # Function for launching the imu node
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
        self.power_cycle_microstrain_driver()
        launch_cmd = ['ros2', 'launch', 'pontus_sensors', 'gx3.launch.py']
        self.subprocess_handle = subprocess.Popen(launch_cmd)  # Start the launch process asynchronously

    def retry_launch(self):
        self.get_logger().info("Retrying to launch sensor driver...")
        self.launch_imu_node()

    def __del__(self):
        self.cleanup()

    def cleanup(self):
        # Ensure that all processies have ended
        if self.redundant_timer:
            self.redundant_timer.cancel()
        if self.timer:
            self.timer.cancel()
        # self.get_logger().info("Trying to deactive lifecycle node")
        # subprocess.run(["ros2", "lifecycle", "set", "/microstrain_inertial_driver", "shutdown"])
        # self.get_logger().info("Successfull deactivation lifecycle node")
        if self.subprocess_handle:
            self.subprocess_handle.terminate()
            self.subprocess_handle.wait()
            if self.subprocess_handle.poll() is None:
                self.get_logger().info("Forcibly terminating the process...")
                self.subprocess_handle.kill()
        os.system("killall microstrain_inertial_driver")

    def destroy_node(self):
        self.cleanup()
        super().destroy_node()

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

