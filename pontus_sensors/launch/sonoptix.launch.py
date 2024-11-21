import os
import ament_index_python
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node, SetRemap
from launch.substitutions import LaunchConfiguration


from launch import LaunchDescription
from launch_ros.actions import Node

_PONTUS_SONOPTIX_PARAMS_FILE = os.path.join(
  ament_index_python.packages.get_package_share_directory('pontus_sensors'),
  'config',
  'sonoptix.yaml'
)

def generate_launch_description():
    return LaunchDescription([
        # Launch the imu_monitor_node
        Node(
            package='pontus_sensors',
            executable='sonoptix_driver.py',
            name='sonoptix_driver',
            parameters=[_PONTUS_SONOPTIX_PARAMS_FILE],
            output='screen',
        ),
    ])
