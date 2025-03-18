from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
import sys
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    stereo_share = get_package_share_directory('pontus_ros_stereo_image_proc')
    _PONTUS_ROS_STEREO_IMAGE_PROC_LAUNCH_FILE = os.path.join(stereo_share,
                                                             'launch',
                                                             'stereo_image_proc.launch.py')
    pkg_share = get_package_share_directory('pontus_perception')
    front_left_camera_model = 'model.pt'
    front_right_camera_model = 'model.pt'
    auv_config_str = 'auv'
    for arg in sys.argv:
        if arg.startswith('auv:='):
            auv_config_str = arg.split(':=')[1]

    return LaunchDescription([
        Node(
            package='pontus_perception',
            executable='yolo',
            name='camera_2_yolo',
            parameters=[{
                'auv': auv_config_str,
                'model_path': os.path.join(pkg_share,
                                           'yolo',
                                           auv_config_str,
                                           front_left_camera_model)
            }],
            remappings=[
                ('input', '/pontus/camera_2/image_rect_color'),
                ('results', '/pontus/camera_2/yolo_results'),
                ('yolo_debug', '/pontus/camera_2/yolo_debug')
            ]
        ),
        # Not needed if using disparity map
        # Node(
        #     package='pontus_perception',
        #     executable='yolo',
        #     name='camera_3_yolo',
        #     parameters=[{
        #         'auv': auv_config_str,
        #         'model_path' : os.path.join(pkg_share, 'yolo', auv_config_str, front_right_camera_model)
        #     }],
        #     remappings=[
        #         ('input', '/pontus/camera_3/image_rect_color'),
        #         ('results', '/pontus/camera_3/yolo_results'),
        #         ('yolo_debug', '/pontus/camera_3/yolo_debug')
        #     ]
        # ),
        Node(
            package='pontus_perception',
            executable='yolo_pose_detection',
            name='yolo_pose_detection',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_PONTUS_ROS_STEREO_IMAGE_PROC_LAUNCH_FILE),
            launch_arguments={
                'left_namespace': '/pontus/camera_2',
                'right_namespace': '/pontus/camera_3',
                'num_disparities' : '16',
                'window_size' : '7',
                'prefilter_cap' : '31',
                'texture_threshold' : '10',
                'uniqueness_ratio' : '5',
                'speckle_window_size' : '200',
                'speckle_range' : '64',
            }.items()
        ),
    ])
