import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import launch.logging  # Import the launch logger

def generate_tuning_commands(context, *args, **kwargs):
    # Initialize the launch logger for this specific script
    logger = launch.logging.get_logger('camera_tuner')

    # Extract the actual string values passed in via command line
    camera_val = LaunchConfiguration('camera').perform(context)
    setting_val = LaunchConfiguration('setting').perform(context)

    valid_cameras = ["left", "right", "facing_down", "tilted_down"]
    
    # Determine which cameras to target
    if camera_val == "all":
        cams_to_update = valid_cameras
    elif camera_val in valid_cameras:
        cams_to_update = [camera_val]
    else:
        # Use logger.error instead of print
        logger.error(f"Invalid camera '{camera_val}'. Valid options: 'all' or {valid_cameras}")
        return []

    processes = []
    
    # Generate an ExecuteProcess action for each targeted camera
    for cam in cams_to_update:
        cmd = ['v4l2-ctl', '-d', f'/dev/camera_{cam}', f'--set-ctrl={setting_val}']
        
        processes.append(
            ExecuteProcess(
                cmd=cmd,
                output='screen',
                name=f"tune_{cam}"
            )
        )
        # Use logger.info instead of print
        logger.info(f"Queued command: {' '.join(cmd)}")

    return processes

def generate_launch_description():
    return LaunchDescription([
        # Argument for which camera to target
        DeclareLaunchArgument(
            'camera',
            default_value='all',
            description="Target camera: 'left', 'right', 'facing_down', 'tilted_down', or 'all'"
        ),
        # Argument for the specific setting to change
        DeclareLaunchArgument(
            'setting',
            default_value='saturation=60',
            description="The v4l2-ctl setting string (e.g., 'hue=0', 'brightness=50')"
        ),
        # OpaqueFunction allows us to run Python logic
        OpaqueFunction(function=generate_tuning_commands)
    ])