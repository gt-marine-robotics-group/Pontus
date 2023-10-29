import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_pontus_sim = get_package_share_directory('pontus_sim')
    bridge_config_path = os.path.join(pkg_pontus_sim, 'config','odom_bridge.yml')

    # Bridge
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_path}],
        output='screen'
    )

    return LaunchDescription([
        odom_bridge])
