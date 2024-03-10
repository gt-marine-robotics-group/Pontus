from ament_index_python import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    controller_share = get_package_share_directory('pontus_controller')
    description_share = get_package_share_directory('pontus_description')

    gazebo_arg = DeclareLaunchArgument('gazebo', default_value='False')
    gazebo_config = LaunchConfiguration('gazebo', default='False')

    return LaunchDescription([
        gazebo_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_share, 'launch', 'spawn.launch.py'),
            ),
            launch_arguments={'gazebo': gazebo_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(controller_share, 'launch', 'vel_control.launch.py')
            )
        )
    ])