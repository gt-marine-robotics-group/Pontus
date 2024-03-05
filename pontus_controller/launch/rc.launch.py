from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_pontus_controller = get_package_share_directory('pontus_controller')
    params = os.path.join(pkg_pontus_controller,'config','joystick.yaml')

    return LaunchDescription([
        Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name = 'teleop_node',
            parameters=[{'ros__parameters': params}],
        ),
        Node(
            package='joy',
            executable='joy_node',
            parameters=[{'ros__parameters': params}],
        )               
    ])