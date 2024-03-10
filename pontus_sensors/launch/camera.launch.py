from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
import sys
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pontus_sensors_share = get_package_share_directory('pontus_sensors')
    pontus_perception_share = get_package_share_directory('pontus_perception')
    oakd_share = get_package_share_directory('depthai_ros_driver')

    ld = list()

    with open(f'{pontus_perception_share}/config/auv/camera_config.yaml', 'r') \
        as stream:
        camera_data = yaml.safe_load(stream)

    for i, topic in enumerate(camera_data['camera_config']['all_camera_base_topics']):
        cam_type = camera_data['camera_config']['all_camera_types'][i]

        if cam_type == 'oakd':
            ld.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(oakd_share, 'launch/camera.launch.py')),
                )
            )
    
    for i, frame in enumerate(camera_data['camera_config']['all_camera_frames']):
        ld.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'base_link_to_{frame}',
                arguments=[
                    *(str(d) for d in camera_data['camera_config']['all_camera_transforms'][i]),
                    '0', '0', '0', frame, 'base_link'
                ]
            )
        )

    for i, topic in enumerate(camera_data['camera_config']['all_camera_base_topics']):
        ld.append(
            Node(
                package='pontus_sensors',
                executable='camera_republish.py',
                name=f'{topic[topic.rfind("/") + 1:]}_republish',
                remappings=[
                    ('input', f'{topic}/image_raw'),
                    ('output', f'{topic}/image_raw/best_effort')
                ]
            )
        )
    
    ld.append(
        Node(
            package='pontus_sensors',
            executable='oak_imu_republish.py'
        )	
    )

    return LaunchDescription(ld)
