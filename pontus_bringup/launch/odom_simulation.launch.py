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
        default_value = 'prequal.world'
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

    odom_bridge = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('pontus_sim'),
                'launch',
                'odom_bridge.launch.py'
            ])
        )
    )

    controls = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('pontus_controller'),
                'launch',
                'pos_control.launch.py'
            ])
        ),
        launch_arguments={
            'sim': 'true',
        }.items()
    )

    return LaunchDescription([
        world_arg,
        static_arg,
        gzsim,
        spawn_vehicle,
        odom_bridge,
        controls
    ])