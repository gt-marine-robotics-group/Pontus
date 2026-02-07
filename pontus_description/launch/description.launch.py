from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import os
import pathlib
import xacro
import xml.etree.ElementTree as ET

def generate_launch_description():

    # Set up launch arguments
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value = 'false'
    )
    sim = LaunchConfiguration('sim')

    static_arg = DeclareLaunchArgument(
        'static',
        default_value = 'false'
    )
    static = LaunchConfiguration('static')

    # Convert Xacro to URDF
    xacro_file = os.path.join(
        get_package_share_directory('pontus_description'),
        'models/pontus',
        'pontus.xacro'
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
    def create_urdf_nodes(context):
        xacro_args = {'sim': context.launch_configurations['sim'], 'static': context.launch_configurations['static']}
        urdf = xacro.process(xacro_file, mappings=xacro_args)

        # parse vehicle parameters out of the URDF file
        vehicle_parameters = {}
        try:
            root = ET.fromstring(urdf)
            base_link = root.find("./link[@name='base_link']")
            intertia_element = base_link.find("inertial").find("inertia")

            for param in base_link.find("params").items():
                vehicle_parameters[param[0]] = float(param[1])

            for attrib in intertia_element.items():
                vehicle_parameters[attrib[0]] = float(attrib[1])

        except Exception as e:
            print("Failed to parse URDF for vehicle parameters: ")
            print(e)

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

        vehicle_parameter_server = Node(
            name = 'vehicle_parameter_server',
            package = 'pontus_description',
            executable = 'vehicle_parameter_server.py',
            output = 'screen',
            parameters = [vehicle_parameters]
        )

        return [robot_state_publisher, vehicle_parameter_server]

    urdf_nodes = OpaqueFunction(function=create_urdf_nodes)

    odom_to_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments = ["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0.0", "--frame-id", "map", "--child-frame-id", "odom"]
    )

    return LaunchDescription([
        sim_arg,
        static_arg,
        urdf_nodes,
        odom_to_map_tf
    ])