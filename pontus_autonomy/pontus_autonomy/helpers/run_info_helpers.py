from rclpy.clock import Clock  # new for timestamps
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Pose, PoseStamped, Point
from pontus_msgs.msg import CommandMode
from pontus_autonomy.helpers.GoToPoseClient import PoseObj
from pontus_mapping.semantic_map_manager import SemanticObject
from dataclasses import dataclass

import numpy as np

import tf_transformations

DEFAULT_DEPTH_M = -1.2
ROBOT_LENGTH = 0.5   # TODO: measure
ROBOT_WIDTH = 0.3

POOL_WIDTH = 20
POOL_DEPTH = -2.1


@dataclass(frozen=True)
class XY:
    x: float
    y: float


@dataclass
class SemanticMapObject:
    label: SemanticObject
    x: float
    y: float
    depth: float = 0.0

    def get_pose_stamped(self, frame_id: str = "map") -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = Clock().now().to_msg()
        ps.pose.position.x = self.x
        ps.pose.position.y = self.y
        ps.pose.position.z = self.depth
        return ps


@dataclass
class SlalomSet:
    red_slalom: SemanticMapObject
    white_slalom_left: SemanticMapObject
    white_slalom_right: SemanticMapObject

    def get_right_side_midpoint(self) -> XY:
        return XY(
            x=(self.red_slalom.x + self.white_slalom_right.x) / 2.0,
            y=(self.red_slalom.y + self.white_slalom_right.y) / 2.0,
        )


def make_waypoint(
    x: float,
    y: float,
    depth: float = DEFAULT_DEPTH_M,
    yaw_degrees: float = None,
    skip_orientation: bool = True,
    command_mode: CommandMode = CommandMode.POSITION_FACE_TRAVEL,
) -> PoseObj:

    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(depth)

    if yaw_degrees is not None:
        yaw_radians = yaw_degrees * (np.pi / 180)

        quat = tf_transformations.quaternion_from_euler(0, 0, yaw_radians)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

    return PoseObj(
        cmd_pose=pose,
        skip_orientation=skip_orientation,
        command_mode=command_mode
    )
