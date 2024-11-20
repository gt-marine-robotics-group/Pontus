from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import os
import pathlib
import xacro



def generate_launch_description():

    # Set up launch arguments
    # There is no option to set sim false here because
    # This file is only used in sim
    static_arg = DeclareLaunchArgument(
        'static',
        default_value = 'false'
    )
    static = LaunchConfiguration('static')

    gazebo_arg = DeclareLaunchArgument('gazebo', default_value='True')
    gazebo_config = LaunchConfiguration('gazebo', default='True')

    #TODO: publish robot tf

    # Convert Xacro to URDF
    xacro_file = os.path.join(
        get_package_share_directory('pontus_description'),
        'models/pontus',
        'pontus_real.xacro'
    )

    # Build the directories, check for existence
    path = os.path.join(
        get_package_share_directory('pontus_description'),
        'generated',
        'pontus'
    )

    if not pathlib.Path(path).exists():
        try:
            # Create directory if required and sub-directory
            os.makedirs(path)
        except OSError:
            print ("Creation of the directory %s failed" % path)

    output = os.path.join(
        path,
        'pontus_description.urdf'
    )

    if not pathlib.Path(xacro_file).exists():
        exc = 'Launch file ' + xacro_file + ' does not exist'
        raise Exception(exc)

    # Accessing launch arguments as strings is currently very cursed
    def create_robot_state_publisher(context):
      xacro_args = {'sim': 'true', 'static': context.launch_configurations['static']}
      urdf = xacro.process(xacro_file, mappings=xacro_args)

      with open(output, 'w') as file_out:
          file_out.write(urdf)

      # Robot description publisher
      robot_state_publisher = Node(
          name = 'robot_state_publisher',
          package = 'robot_state_publisher',
          executable = 'robot_state_publisher',
          output = 'screen',
          parameters = [{'robot_description': urdf}]
      )

      return [robot_state_publisher]

    robot_state_publisher = OpaqueFunction(function=create_robot_state_publisher)

    # URDF spawner
    args = ['-name', 'pontus', '-topic', 'robot_description']
    spawn = Node(
        package='ros_gz_sim', 
        executable='create', 
        arguments=args, 
        output='screen',
        condition=IfCondition(gazebo_config)
    )

    return LaunchDescription([
        gazebo_arg,
        static_arg,
        robot_state_publisher,
        spawn
    ])
