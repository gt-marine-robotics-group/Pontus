import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    pontus_sensors_share = get_package_share_directory('pontus_sensors')

    config = os.path.join(
        pontus_sensors_share,
        'config',
        'blueview.yaml'
    )

    color_map = os.path.join(
        pontus_sensors_share,
        'colormaps',
        'jet.cmap'
    )

    # Make sure to unset the device parameter in the config file to use this
    file = ""
    '''
    file = os.path.join(
        pontus_sensors_share,
        'data',
        'swimmer.son'
    )
    '''

    blueview = Node(
        package = 'pontus_sensors',
        name = 'ros_blueview_driver',
        executable = 'ros_blueview_driver',
        parameters = [
            config,
            {"color/map_file": color_map},
            {"file": file}
        ]
    )

    ld.add_action(blueview)
    return ld
