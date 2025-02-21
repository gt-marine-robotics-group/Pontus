import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pontus_autonomy',
            executable='gate_information_service',
            name='gate_information_serivce',
            output='screen',
        ),
        Node(
            package='pontus_autonomy',
            executable='prequalification_run_localization',
            name='prequalification_run_localization',
            output='screen',
        ),
    ])
