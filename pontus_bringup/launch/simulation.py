from ament_index_python import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

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
                'sim.py'
            ])
        ),
        launch_arguments = {'world': world}.items()
    )

    spawn_vehicle = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('pontus_description'),
                'launch',
                'spawn.py'
            ])
        ),
        launch_arguments = {'static': static}.items()
    )

    robot_localization = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('pontus_localization'),
                'launch',
                'localization.py'
            ])
        )
    )

    controls = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('pontus_controller'),
                'launch',
                'vel_control.py'
            ])
        )
    )

    return LaunchDescription([
        world_arg,
        static_arg,
        gzsim,
        spawn_vehicle,
        robot_localization,
        controls
    ])
