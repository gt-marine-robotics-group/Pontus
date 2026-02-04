from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    gazebo_arg = DeclareLaunchArgument('gazebo', default_value='True')
    gazebo_config = LaunchConfiguration('gazebo', default='True')

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
        spawn
    ])
