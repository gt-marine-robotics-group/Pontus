from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='pontus_controller',
            executable='thruster_controller',
            output='screen'
        ),
        Node(
            package='pontus_controller',
            executable='velocity_controller',
            output='screen'
        ),
        Node(
            package='pontus_controller',
            executable='position_controller',
            output='screen',
        )
    ])