from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pontus_mapping_share = get_package_share_directory('pontus_mapping')
    # visited_parameters = f'{pontus_mapping_share}/config/visited_parameters.yaml'

    topics_config_arg = DeclareLaunchArgument(
        'topics_config',
        default_value=PathJoinSubstitution([
            get_package_share_directory('pontus_bringup'),
            'config', 'topics.yaml'
        ]),
    )

    return LaunchDescription([
        topics_config_arg,

        Node(
            package='pontus_mapping',
            executable='semantic_map_manager',
            name='semantic_map_manager',
            output='screen',
        ),

        # Node(
        #     package='pontus_mapping',
        #     executable='occupancy_grid_manager',
        #     name='occupancy_grid_manager',
        #     output='screen',
        # ),

        Node(
            package='pontus_mapping',
            executable='cluster_coord',
            name='cluster_coord',
            output='screen',
            parameters=[
                ParameterFile(LaunchConfiguration('topics_config'), allow_substs=True),
            ],
        )
    ])
