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
           package='tf2_ros',
           executable='static_transform_publisher',
           name='imu_to_base',
           # 90, 90, 0
           #arguments=['--qx', '-0.5', '--qy', '-0.5', '--qz', '0.5', '--qw', '0.5', '--frame-id', 'base_link', '--child-frame-id', 'sensor']
           arguments=['0', '0', '0', '1.57', '3.141', '1.57', 'base_link', 'sensor']
	),
       Node(
           package='pontus_sensors',
           executable='imu_republish.py'
       )
    ])
