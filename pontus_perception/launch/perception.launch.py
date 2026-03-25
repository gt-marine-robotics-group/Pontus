# ------ Imports ------

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterFile
from typing import Optional


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('pontus_perception')

    topics_config_arg = DeclareLaunchArgument(
        'topics_config',
        default_value=PathJoinSubstitution([
            get_package_share_directory('pontus_bringup'),
            'config', 'topics.yaml'
        ]),
    )

    # ------  Set whether sim or read hardware ------
    auv_config_str: Optional[str] = 'auv'
    for arg in sys.argv:
        if arg.startswith('auv:='):
            auv_config_str = arg.split(':=')[1]

    # ------ Load the YOLO model ------
    yolo_model = 'model.pt'
    model_path = os.path.join(pkg_share, 'yolo', auv_config_str, yolo_model)


    # ------ Creating Perception Nodes -------

    # --- Front camera YOLO ---
    front_camera_topic = '/pontus/camera_front'
    # front_camera_topic = '/pontus/camera_2'

    front_camera_YOLO_node: Node = Node(
        package='pontus_perception',
        executable='yolo',
        name='front_camera_yolo',
        parameters=[
    ParameterFile(LaunchConfiguration('topics_config'), allow_substs=True),
    {
        'auv': auv_config_str,
        'model_path': model_path,
    }
]
    )

    launch_description: list[Node] = [
        front_camera_YOLO_node
    ]

    return LaunchDescription([
    topics_config_arg,
    *launch_description
])