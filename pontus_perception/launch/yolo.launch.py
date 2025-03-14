from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import sys
import yaml


def generate_launch_description():
    pkg_share = get_package_share_directory('pontus_perception')

    auv_arg = DeclareLaunchArgument('auv')

    auv_config_str = None
    for arg in sys.argv:
        if arg.startswith('auv:='):
            auv_config_str = arg.split(':=')[1]

    camera_data = None
    with open(f'{pkg_share}/config/{auv_config_str}/camera_config.yaml', 'r') as stream:
        camera_data = yaml.safe_load(stream)

    ld = []
    ld.append(auv_arg)

    for topic in camera_data['camera_config']['all_camera_base_topics']:
        ld.append(
            Node(
                package='pontus_perception',
                executable='yolo',
                name=f'perception_YOLO_{topic[topic.rfind("/") + 1:]}',
                parameters=[
                    {'auv': auv_config_str}
                ],
                remappings=[
                    ('input', topic + '/image_rect_color'),
                    ('results', topic + '/yolo_results'),
                    ('yolo_debug', topic + '/yolo_debug')
                ]
            )
        )

    return LaunchDescription(ld)
