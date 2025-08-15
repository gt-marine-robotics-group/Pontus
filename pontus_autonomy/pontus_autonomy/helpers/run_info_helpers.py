from rclpy.clock import Clock  # new for timestamps
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Pose, PoseStamped, Point
from pontus_autonomy.helpers.GoToPoseClient import PoseObj, MovementMethod
from pontus_mapping.semantic_map_manager import SemanticObject
from dataclasses import dataclass

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
    skip_orientation: bool = True,
    movement_method: MovementMethod = MovementMethod.TurnThenForward,
) -> PoseObj:

    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(depth)

    return PoseObj(
        cmd_pose=pose,
        skip_orientation=skip_orientation,
        movement_method=movement_method
    )
