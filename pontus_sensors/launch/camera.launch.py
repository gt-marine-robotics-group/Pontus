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
    
    imu_arg = DeclareLaunchArgument('imu', default_value='gx3')
    imu_config = LaunchConfiguration('imu', default='gx3')

    imu_config_str = 'gx3'
    for arg in sys.argv:
        if arg.startswith('imu:='):
            imu_config_str = arg.split(':=')[1]

    ld = list()
    ld.append(imu_arg)

    with open(f'{pontus_perception_share}/config/camera_config.yaml', 'r') \
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
            # uncomment to use oakd imu
            # if imu_config_str == 'oak': # using oak-d imu
            #     ld.append(Node(
            #         package='virtuoso_sensors',
            #         executable='gx3_republish',
            #         remappings=[
            #             ('/imu/data', '/oak/imu/data')
            #         ]
            #     ))
    
    for i, frame in enumerate(camera_data['camera_config']['all_camera_frames']):
        ld.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'base_link_to_{frame}',
                arguments=[
                    *(str(d) for d in camera_data['camera_config']['all_camera_transforms'][i]),
                    '0', '0', '0', 'base_link', frame
                ]
            )
        )

    for i, topic in enumerate(camera_data['camera_config']['all_camera_base_topics']):
        ld.append(
            Node(
                package='virtuoso_sensors',
                executable='camera_republish.py',
                name=f'{topic[topic.rfind("/") + 1:]}_republish',
                remappings=[
                    ('input', f'{topic}/image_raw'),
                    ('output', f'{topic}/image_raw/best_effort')
                ]
            )
        )

    return LaunchDescription(ld)
