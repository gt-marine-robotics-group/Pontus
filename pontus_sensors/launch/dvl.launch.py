import os
import ament_index_python
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node, SetRemap
from launch.substitutions import LaunchConfiguration


from launch import LaunchDescription
from launch_ros.actions import Node

_PONTUS_DVL_PARAMS_FILE = os.path.join(
  ament_index_python.packages.get_package_share_directory('pontus_sensors'),
  'config',
  'dvl.yaml'
)

def generate_launch_description():
    imu_correction_arg = DeclareLaunchArgument('imu_correction', default_value = 'true')
    imu_correction_config = LaunchConfiguration('imu_correction')

    return LaunchDescription([
        imu_correction_arg,
        Node(
            package='dvl_a50',
            executable='dvl_a50_sensor',
            name='dvl_a50_sensor',
            parameters=[_PONTUS_DVL_PARAMS_FILE],
            output='screen',
        ),
        Node(
            package='pontus_sensors',
            executable='dvl_republish.py',
            name='dvl_republish',
            parameters=[
                {'imu_correction': imu_correction_config},
            ],
            output='screen',
        )
    ])
