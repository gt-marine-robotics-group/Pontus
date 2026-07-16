from ament_index_python import get_package_share_directory
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    sensors_share = get_package_share_directory('pontus_sensors')
    perception_share = get_package_share_directory('pontus_perception')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sensors_share, 'launch', 'low_light.launch.py'),
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(perception_share, 'launch', 'perception.launch.py'),
            )
        ),
    ])