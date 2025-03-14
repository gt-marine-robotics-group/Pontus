from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pontus_mapping_share = get_package_share_directory('pontus_mapping')
    visited_parameters = f'{pontus_mapping_share}/config/visited_parameters.yaml'

    return LaunchDescription([
        Node(
            package='pontus_mapping',
            executable='exploration_map_manager',
            name='exploration_map_manager',
            output='screen',
            parameters=[visited_parameters]
        )
    ])
