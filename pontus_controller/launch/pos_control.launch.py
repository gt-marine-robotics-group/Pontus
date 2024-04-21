from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_pontus_controller = get_package_share_directory('pontus_controller')

    return LaunchDescription([
        Node(
            package='pontus_controller',
            executable='thruster_controller',
            output='screen'
        ),
        Node(
            package='pontus_controller',
            executable='position_controller',
            output='screen'
        ),
        Node(
            package='pontus_controller',
            executable='path_follower',
            output='screen'
        ),
        Node(
            package='pontus_controller',
            executable='velocity_controller',
            output='screen'
        )
    ])
