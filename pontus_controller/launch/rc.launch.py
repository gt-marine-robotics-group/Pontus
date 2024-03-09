from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_pontus_controller = get_package_share_directory('pontus_controller')

    auv_arg = DeclareLaunchArgument('auv', default_value='auv')
    auv_config = LaunchConfiguration('auv', default='auv')

    params = (pkg_pontus_controller, '/config/', auv_config, '/joystick.yaml')

    return LaunchDescription([
        auv_arg,
        Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name = 'teleop_node',
            parameters=[params],
        ),
        Node(
            package='joy',
            executable='joy_node',
            parameters=[params],
        )               
    ])