from ament_index_python import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from pontus_msgs.msg import CommandMode
import os

def generate_launch_description():

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='prequal.world'
    )
    world = LaunchConfiguration('world')

    static_arg = DeclareLaunchArgument(
        'static',
        default_value='false'
    )
    static = LaunchConfiguration('static')

    default_command_mode_arg = DeclareLaunchArgument(
        'default_command_mode',
        default_value= str(CommandMode.POSITION_FACE_TRAVEL)
    )
    default_command_mode = LaunchConfiguration('default_command_mode')


    localization_share = get_package_share_directory('pontus_localization')

    description = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('pontus_description'),
                'launch',
                'description.launch.py'
            ])
        ),
        launch_arguments={
            'static': static,
            'sim': 'true'
        }.items()
    )

    gzsim = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('pontus_sim'),
                'launch',
                'sim.launch.py'
            ])
        ),
        launch_arguments={'world': world}.items()
    )

    spawn_vehicle = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('pontus_sim'),
                'launch',
                'spawn.launch.py'
            ])
        ),
        launch_arguments={'static': static}.items()
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

    localization = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_share, 'launch',
                         'localization.launch.py')
        ),
        launch_arguments={'auv': 'sim'}.items()
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
            'default_command_mode': default_command_mode
        }.items()
    )

    dvl_republisher = Node(
        package='pontus_sensors',
        executable='dvl_republish_sim.py',
    )

    return LaunchDescription([
        world_arg,
        static_arg,
        default_command_mode_arg,
        description,
        gzsim,
        spawn_vehicle,
        odom_bridge,
        controls,
        dvl_republisher,
        localization
    ])
