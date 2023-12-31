from ament_index_python import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    auv_arg = DeclareLaunchArgument('auv')
    auv_config = LaunchConfiguration('auv')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value = 'underwater.world'
    )
    world = LaunchConfiguration('world')

    static_arg = DeclareLaunchArgument(
        'static',
        default_value = 'false'
    )
    static = LaunchConfiguration('static')


    gzsim = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('pontus_sim'),
                'launch',
                'sim.launch.py'
            ])
        ),
        launch_arguments = {'world': world}.items()
    )

    spawn_vehicle = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('pontus_description'),
                'launch',
                'spawn.launch.py'
            ])
        ),
        launch_arguments = {'static': static}.items()
    )

    robot_localization = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('pontus_localization'),
                'launch',
                'localization.launch.py'
            ])
        ),
        launch_arguments={'auv': auv_config}.items()
    )

    controls = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('pontus_controller'),
                'launch',
                'vel_control.launch.py'
            ])
        )
    )

    return LaunchDescription([
        auv_arg,
        world_arg,
        static_arg,
        gzsim,
        spawn_vehicle,
        robot_localization,
        controls
    ])
