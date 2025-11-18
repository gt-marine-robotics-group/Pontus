from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from pontus_msgs.msg import CommandMode

def generate_launch_description():

    default_command_mode_arg = DeclareLaunchArgument(
        'default_command_mode',
        default_value = str(CommandMode.ESTOP)
    )
    default_command_mode = LaunchConfiguration('default_command_mode')

    return LaunchDescription([
        default_command_mode_arg,
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
            parameters=[{'default_command_mode': LaunchConfiguration('default_command_mode')}]
        )
    ])