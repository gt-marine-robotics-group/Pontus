# pool_mapping_bringup.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    # --- Arguments ---

    map_yaml = DeclareLaunchArgument(
        "map_yaml",
        default_value=PathJoinSubstitution([
            EnvironmentVariable("HOME"),
            "pontus_ws", "pool_image_visualizer", "pool.yaml"
        ]),
        description="Path to the pool map YAML (image + resolution + origin).",
    )

    map_frame = DeclareLaunchArgument(
        "map_frame",
        default_value="map",
        description="Frame ID to publish the OccupancyGrid in (e.g., 'map' or 'pool_map').",
    )

    # --- Nodes ---

    # Map server (lifecycle node)
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "yaml_filename": LaunchConfiguration("map_yaml"),
            "frame_id": LaunchConfiguration("map_frame"),
        }],
    )

    pontus_mapping_share = get_package_share_directory("pontus_mapping")
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pontus_mapping_share, "launch", "mapping.launch.py")
        )
    )

    # 4) Seed the semantic map (give a small delay so other nodes are up)
    semantic_map_seeder = TimerAction(
        period=3.0,
        actions=[Node(
            package="pontus_autonomy",
            executable="semantic_map_seeder",
            name="semantic_map_seeder",
            output="screen",
        )]
    )

    # Lifecycle manager to auto-configure + activate map_server
    lifecycle_mgr = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["map_server"],
            "bond_timeout": 0.0,
            "attempts": 3,
        }],
    )

    waypoint_visualizer = TimerAction(
        period=2.0,  # delay is optional
        actions=[Node(
            package="pontus_autonomy",
            executable="run_info_waypoint_visualizer",
            name="run_info_waypoint_visualizer",
            output="screen",
            # If you want to override defaults, uncomment and edit:
            # parameters=[{
            #     "frame_id": "map",
            #     "topic": "/run_waypoints_markers",
            #     "line_width": 0.08,
            #     "point_diameter": 0.20,
            # }],
        )]
    )

    return LaunchDescription([
        map_yaml, map_frame,
        map_server,
        mapping_launch,
        semantic_map_seeder,
        lifecycle_mgr,
        waypoint_visualizer,
    ])
