import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import sys

def generate_launch_description():
    pontus_perception_share = get_package_share_directory('pontus_perception')
    auv_arg = DeclareLaunchArgument('auv', default_value='auv')

    auv_config_str = None
    for arg in sys.argv:
        if arg.startswith('auv:='):
            auv_config_str = arg.split(':=')[1]

    gate_detection_config_file = f'{pontus_perception_share}/config/{auv_config_str}/gate_detection.yaml'

    return LaunchDescription([
        Node(
            package='pontus_perception',
            executable='gate_detection',
            name='gate_detection',
            output='screen',
            parameters=[gate_detection_config_file],
        ),
    ])
