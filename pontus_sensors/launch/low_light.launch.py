import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pontus_sensors_share = get_package_share_directory('pontus_sensors')

    launch_descriptions = []

    camera_names = [
        "left",
        "right",
        # "facing_down",
        # "tilted_down"
    ]

    for name in camera_names:
        camera_node = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name=f"{name}_camera",
            output='screen',
            parameters=[f'{pontus_sensors_share}/config/low_light_camera_{name}.yaml'],
            remappings=[
                # ('/image_raw', f'/pontus/camera_{name}/image_raw'), # Theoretically these are disabled for efficiency
                ('/image_raw/compressed', f'/pontus/camera_{name}/image_raw/compressed'),
                ('/camera_info', f'/pontus/camera_{name}/camera_info')
            ]
        )

        cam_settings = [
            "white_balance_automatic=1",
            # "white_balance_temperature=5500",
            "brightness=50",
            "saturation=100",
            "hue=-15",
            "gamma=0",
            "gain=0"
        ]
        cam_setting_string = "--set-ctrl="
        for setting in cam_settings:
            cam_setting_string += f"{setting},"

        # Could also set exposure dynamic framerate to stop lowering framerate in low light conditions
        set_v4l_configs = RegisterEventHandler(
            OnProcessStart(
                target_action = camera_node,
                on_start = [
                    ExecuteProcess(
                        cmd=['v4l2-ctl', '-d', f'/dev/camera_{name}', cam_setting_string])
                ]
            )
        )

        print('v4l2-ctl ' + '-d ' + f'/dev/camera_{name} ' + cam_setting_string)

        launch_descriptions.append(camera_node)
        launch_descriptions.append(set_v4l_configs)

    return LaunchDescription(launch_descriptions)
