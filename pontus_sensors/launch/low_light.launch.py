import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pontus_sensors_share = get_package_share_directory('pontus_sensors')
    left_config_file = f'{pontus_sensors_share}/config/low_light_camera_left.yaml'
    right_config_file = f'{pontus_sensors_share}/config/low_light_camera_right.yaml'
    left_middle_config_file = f'{pontus_sensors_share}/config/low_light_camera_middle_left.yaml'
    down_config_file = f'{pontus_sensors_share}/config/low_light_camera_down.yaml'
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name="left_camera",
            output='screen',
            parameters=[left_config_file],
            remappings=[
                ('/image_raw', '/pontus/camera_2/image_raw'),
                ('/image_raw/compressed', '/pontus/camera_2/image_raw/compressed'),
                ('/image_raw/compressedDepth', '/pontus/camera_2/image_raw/compressedDepth'),
                ('/image_raw/theora', '/pontus/camera_2/image_raw/theora'),
                ('/camera_info', '/pontus/camera_2/camera_info')
                ]
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name="right_camera",
            output='screen',
            parameters=[right_config_file],
            remappings=[
                ('/image_raw', '/pontus/camera_3/image_raw'),
                ('/image_raw/compressed', '/pontus/camera_3/image_raw/compressed'),
                ('/image_raw/compressedDepth', '/pontus/camera_3/image_raw/compressedDepth'),
                ('/image_raw/theora', '/pontus/camera_3/image_raw/theora'),
                ('/camera_info', '/pontus/camera_3/camera_info')
                ]
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name="side_camera",
            output='screen',
            parameters=[left_middle_config_file],
            remappings=[
                ('/image_raw', '/pontus/camera_4/image_raw'),
                ('/image_raw/compressed', '/pontus/camera_4/image_raw/compressed'),
                ('/image_raw/compressedDepth', '/pontus/camera_4/image_raw/compressedDepth'),
                ('/image_raw/theora', '/pontus/camera_4/image_raw/theora'),
                ('/camera_info', '/pontus/camera_4/camera_info')
                ]
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name="down_camera",
            output='screen',
            parameters=[down_config_file],
            remappings=[
                ('/image_raw', '/pontus/camera_1/image_raw'),
                ('/image_raw/compressed', '/pontus/camera_1/image_raw/compressed'),
                ('/image_raw/compressedDepth', '/pontus/camera_1/image_raw/compressedDepth'),
                ('/image_raw/theora', '/pontus/camera_1/image_raw/theora'),
                ('/camera_info', '/pontus/camera_1/camera_info')
                ]
        )
    ])
