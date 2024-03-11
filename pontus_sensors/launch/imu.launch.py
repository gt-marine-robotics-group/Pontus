import os

import ament_index_python

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node, SetRemap


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the imu_monitor_node
        Node(
            package='pontus_sensors',
            executable='imu_monitor.py',
            name='imu_monitor',
        ),
        Node(
            package='pontus_sensors',
            executable='imu_republish.py',
        )
    ])