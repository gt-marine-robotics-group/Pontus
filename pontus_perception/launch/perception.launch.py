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
                ('yolo_debug', '/pontus/camera_2/yolo_debug'),
                ('yolo_debug/compressed', '/pontus/camera_2/yolo_debug/compressed')
            ]
        ),
        Node(
            package='pontus_perception',
            executable='cylinder_shape_detection',
            name='cylinder_shape_detection',
            output='screen'
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
        #         ('yolo_debug', '/pontus/camera_3/yolo_debug'),
        #         ('yolo_debug/compressed', '/pontus/camera_3/yolo_debug/compressed')
        #     ]
        # ),
        # Node(
        #     package='pontus_perception',
        #     executable='yolo_pose_detection',
        #     name='yolo_pose_detection',
        #     output='screen',
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_PONTUS_ROS_STEREO_IMAGE_PROC_LAUNCH_FILE),
            launch_arguments={
                'left_namespace': '/pontus/camera_2',
                'right_namespace': '/pontus/camera_3',
                'num_disparities' : '32',
                'window_size' : '5',
                'prefilter_cap' : '27',
                'texture_threshold' : '3',
                'uniqueness_ratio' : '5',
                'speckle_window_size' : '200',
                'speckle_range' : '32',
            }.items()
        ),
    ])
"""
num_disparities (int) :
* Higher value means less FOV but better detections further away

window_size (int) :
* Higher means more finer detail

prefilter_cap (int) :
Limits the prefiltering done before the disparity calculation. 
* Higher value leads to less filtering, lower value leads to more filtering (removing noise)

texture_threshold (int) :
defines a threshold that specfiies how mcuh texture must be present for stereo block matching
to occur. Specifically for low texture regions.
* Lower values mean that the algorithm is more likely to match low-texture regions

uniqueness_ratio (int) :
Basically represents the confidence of a disparity match
* Higher values means more confidence

speckle_window_size (int) :
Controls size of the region in which small disparities are allowed
* Higher value reduces more noise

speckle_range (int):
Maximum disparite difference within the speckle window for the dispartiy values to be 
considered the same region. Dispartieis that are more than this value are treated as noise
and removed
* Lower value removes more speckle
"""