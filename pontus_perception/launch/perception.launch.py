# ------ Imports ------

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterFile
from typing import Optional


def generate_launch_description() -> LaunchDescription:
    topics_config_arg = DeclareLaunchArgument(
        'topics_config',
        default_value=PathJoinSubstitution([
            get_package_share_directory('pontus_bringup'),
            'config', 'topics.yaml'
        ]),
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Override path to YOLO model (.pt or .engine). '
                    'If empty, falls back to the package default.',
    )

    force_compressed_arg = DeclareLaunchArgument(
        'force_compressed',
        default_value='false',
        description='Subscribe directly to compressed image topics, skipping raw.',
    )

    # ------  Set whether sim or read hardware ------
    auv_config_str: Optional[str] = 'auv'
    for arg in sys.argv:
        if arg.startswith('auv:='):
            auv_config_str = arg.split(':=')[1]

    # ------ Creating Perception Nodes -------

    multi_cam_yolo_node: Node = Node(
        package='pontus_perception',
        executable='yolo',
        name='multi_cam_yolo',
        parameters=[
            ParameterFile(LaunchConfiguration('topics_config'), allow_substs=True),
            {
                'auv': auv_config_str,
                'model_path': LaunchConfiguration('model_path'),
                'force_compressed': LaunchConfiguration('force_compressed'),
            },
        ],
    )

    launch_description: list[Node] = [
        multi_cam_yolo_node,
    ]

    return LaunchDescription([
        topics_config_arg,
        model_path_arg,
        force_compressed_arg,
        *launch_description
    ])
