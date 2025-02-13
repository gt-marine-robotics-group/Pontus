import os

import ament_index_python

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node, SetRemap

# Path to the launch files and directories that we will use
_STEREO_IMAGE_PROC_LAUNCH_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('stereo_image_proc'), 'launch', 'stereo_image_proc.launch.py')

def generate_launch_description():
  return LaunchDescription([
    # Stereo Image Proc Node
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(_STEREO_IMAGE_PROC_LAUNCH_FILE),
      launch_arguments={
        'left_namespace': '/pontus/camera_2',
        'right_namespace': '/pontus/camera_3',
        'use_color': 'false',
        'approximate_sync': 'true',
        'correlation_window_size': '9',

      }.items()
    ),
    Node(
      package='pontus_perception',
      executable='point_cloud_downsampling',
      name='point_cloud_downsampling',
      output='screen'
    )
  ])