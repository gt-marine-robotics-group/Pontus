from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('pontus_localization')

    auv_arg = DeclareLaunchArgument('auv')
    auv_config = LaunchConfiguration('auv')

    imu_correction_arg = DeclareLaunchArgument('imu_correction', default_value = 'true')
    imu_correction_config = LaunchConfiguration('imu_correction')

    depth_correction_arg = DeclareLaunchArgument('depth_correction', default_value = 'false')
    depth_correction_config = LaunchConfiguration('depth_correction')

    robot_localization_file_path = (pkg_share, '/config/', auv_config, '/ekf.yaml')

    return LaunchDescription([
        auv_arg,
        imu_correction_arg,
        depth_correction_arg,

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[robot_localization_file_path],
            remappings=[
                ("/odometry/filtered", "/pontus/odometry"),
            ]
        ),
        # Node(
        #     package='pontus_localization',
        #     executable='odom_correction_node',
        #     name='odom_correction_node',
        #     parameters=[
        #         {'imu_correction': imu_correction_config},
        #         {'depth_correction': depth_correction_config}
        #     ]
        # )
    ])
