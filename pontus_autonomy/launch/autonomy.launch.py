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
            executable='prequal_loc',
            name='prequal_loc',
            output='screen',
        ),
    ])
