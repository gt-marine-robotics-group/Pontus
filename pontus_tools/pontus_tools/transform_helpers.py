"""Helper utilities shared across Pontus packages"""

# ------- Libraries -------
from __future__ import annotations

from typing import Optional

from rclpy.logging import get_logger
from rclpy.time import Time

from geometry_msgs.msg import Pose, PoseStamped

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

LOGGER = get_logger(__name__)

def convert_to_map_frame(
    pose_stamped: PoseStamped,
    tf_buffer: tf2_ros.Buffer,
    target_frame: str = 'map',
) -> Optional[Pose]:
    """Transform a pose into the target frame (defaults to ``map``).

    Args:
        pose_stamped: Pose to transform.
        tf_buffer: TF2 buffer used to look up transforms.
        target_frame: Desired frame for the resulting pose.

    Returns:
        The transformed pose in the target frame, or ``None`` if the
        transform was unavailable.
    """
    try:
        transform = tf_buffer.lookup_transform(
            target_frame,
            pose_stamped.header.frame_id,
            Time())
    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as exc:
        LOGGER.warning(
            'Failed to lookup transform from %s to %s: %s',
            pose_stamped.header.frame_id,
            target_frame,
            exc,
        )
        return None

    transformed = do_transform_pose(pose_stamped, transform)
    return transformed.pose if hasattr(transformed, 'pose') else transformed