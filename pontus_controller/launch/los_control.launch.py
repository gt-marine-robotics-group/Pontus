from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():

    pkg_pontus_controller = get_package_share_directory('pontus_controller')

    auv_arg = DeclareLaunchArgument('auv', default_value='auv')
    auv_config = LaunchConfiguration('auv', default='auv')
    
    sim_arg = DeclareLaunchArgument('sim', default_value='false')
    sim_config = LaunchConfiguration('sim', default='false')

    return LaunchDescription([
        auv_arg,
        sim_arg,
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
            executable='firmware_cmds',
            condition=IfCondition(PythonExpression(
                ["'", auv_config, "' == 'auv'"]
            ))
        ),
        Node(
            package='pontus_controller',
            executable='LOS_controller',
            output='screen',
        ),
    ])