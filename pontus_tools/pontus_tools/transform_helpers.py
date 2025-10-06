# ------- Libraries -------
import rclpy

from geometry_msgs.msg import Pose, PoseStamped, Twist

import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import tf_transformations

def convert_to_map_frame(position: PoseStamped, tf_buffer: tf2_ros.buffer) -> Pose:
    """
    Convert a point in relation to the body frame to the map frame.

    Args:
    ----
    position (PoseStamped): the point to transform
    tf_buffer (tf2_ros.buffer): tf buffer to get transform

    Return:
    ------
    Pose: the pose in the map frame

    """
    try:
        transform = tf_buffer.lookup_transform(
            'map',
            position.header.frame_id,
            rclpy.time.Time()
        )
    except Exception as e:
        rclpy.get_logger().info(
            f'Exception {e}. Failed to get map transfrom. Skipping')
        return None

    pose_map_frame = do_transform_pose(position.pose, transform)
    return pose_map_frame