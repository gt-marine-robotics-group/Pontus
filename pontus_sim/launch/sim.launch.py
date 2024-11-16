import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_pontus_sim = get_package_share_directory('pontus_sim')

    #TODO: publish world frame, add bridges for all necessary topics
    world_arg = DeclareLaunchArgument(
        'world',
        default_value = 'prequal.world'
    )
    world = LaunchConfiguration('world')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r ', world]
        }.items(),
    )


    bridge_config_path = os.path.join(pkg_pontus_sim, 'config','bridge.yml')

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_path}],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gz_sim,
        bridge])
