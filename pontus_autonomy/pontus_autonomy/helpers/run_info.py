from rclpy.clock import Clock  # new for timestamps
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Pose, PoseStamped, Point
from pontus_autonomy.helpers.GoToPoseClient import PoseObj, MovementMethod
from pontus_mapping.semantic_map_manager import SemanticObject
from dataclasses import dataclass

DEFAULT_DEPTH_M = -1.2
ROBOT_LENGTH = 1.5   # TODO: measure
ROBOT_WIDTH = 0.75


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
        # orientation left at zero; caller may set heading
        return ps


# ------- Semantic Map Info -------
GATE_CENTER_X = 0.0
GATE_CENTER_Y = 0.0

gate_left = SemanticMapObject(
    label=SemanticObject.GateLeft,
    x=GATE_CENTER_X - 1.0,
    y=GATE_CENTER_Y + 0.0
)

gate_right = SemanticMapObject(
    label=SemanticObject.GateRight,  # FIX: use correct enum
    x=GATE_CENTER_X + 1.0,
    y=GATE_CENTER_Y + 0.0
)

# ------ Slalom -------


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


# --- Slalom Set 1 ---
RED_SLALOM_1_X = GATE_CENTER_X + 2.0
RED_SLALOM_1_Y = GATE_CENTER_Y + 3.0

slalom_set_1 = SlalomSet(
    red_slalom=SemanticMapObject(
        label=SemanticObject.Red_Slalom,
        x=RED_SLALOM_1_X,
        y=RED_SLALOM_1_Y
    ),

    white_slalom_left=SemanticMapObject(
        label=SemanticObject.White_Slalom,
        x=RED_SLALOM_1_X - 1.0,
        y=RED_SLALOM_1_Y + 0.0
    ),

    white_slalom_right=SemanticMapObject(
        label=SemanticObject.White_Slalom,
        x=RED_SLALOM_1_X + 1.0,
        y=RED_SLALOM_1_Y + 0.0
    )
)


# --- Slalom Set 2 ---
RED_SLALOM_2_X = RED_SLALOM_1_X + 1.0
RED_SLALOM_2_Y = RED_SLALOM_1_Y + 2.0

slalom_set_2 = SlalomSet(
    red_slalom=SemanticMapObject(
        label=SemanticObject.Red_Slalom,
        x=RED_SLALOM_2_X,
        y=RED_SLALOM_2_Y
    ),

    white_slalom_left=SemanticMapObject(
        label=SemanticObject.White_Slalom,
        x=RED_SLALOM_2_X - 1.0,
        y=RED_SLALOM_2_Y + 0.0
    ),

    white_slalom_right=SemanticMapObject(
        label=SemanticObject.White_Slalom,
        x=RED_SLALOM_2_X + 1.0,
        y=RED_SLALOM_2_Y + 0.0
    )
)


# --- Slalom Set 3 ---
RED_SLALOM_3_X = RED_SLALOM_2_X + 1.0
RED_SLALOM_3_Y = RED_SLALOM_2_Y + 2.0

slalom_set_3 = SlalomSet(
    red_slalom=SemanticMapObject(
        label=SemanticObject.Red_Slalom,
        x=RED_SLALOM_3_X,
        y=RED_SLALOM_3_Y
    ),

    white_slalom_left=SemanticMapObject(
        label=SemanticObject.White_Slalom,
        x=RED_SLALOM_3_X - 1.0,
        y=RED_SLALOM_3_Y + 0.0
    ),

    white_slalom_right=SemanticMapObject(
        label=SemanticObject.White_Slalom,
        x=RED_SLALOM_3_X + 1.0,
        y=RED_SLALOM_3_Y + 0.0
    )
)


# ------- Octagon ------
octagon = SemanticMapObject(
    label=SemanticObject.Octagon,
    x=GATE_CENTER_X - 2.5,
    y=GATE_CENTER_Y + 18.0
)

semantic_map: list[SemanticMapObject] = [
    gate_left,
    gate_right,
    slalom_set_1.red_slalom, slalom_set_1.white_slalom_left, slalom_set_1.white_slalom_right,
    slalom_set_2.red_slalom, slalom_set_2.white_slalom_left, slalom_set_2.white_slalom_right,
    slalom_set_3.red_slalom, slalom_set_3.white_slalom_left, slalom_set_3.white_slalom_right,
    octagon
]

slalom_midpoint_1: XY = slalom_set_1.get_right_side_midpoint()
slalom_midpoint_2: XY = slalom_set_2.get_right_side_midpoint()
slalom_midpoint_3: XY = slalom_set_3.get_right_side_midpoint()

# ------- Waypoints ------


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


waypoints_list: list = [
    make_waypoint(0, 0),
    make_waypoint(1, 0),
    make_waypoint(1, 1),
    make_waypoint(1, 0),
    make_waypoint(0, 0, depth=0.0)
]

# waypoints_list: list[PoseObj] = [
#     # Through the gate (slightly right of gate center)
#     make_waypoint(gate_right.x - (ROBOT_WIDTH / 2.0) -
#                   0.1, gate_right.y + 0.2),

#     # Slalom Task
#     make_waypoint(slalom_midpoint_1.x, slalom_midpoint_1.y),
#     make_waypoint(slalom_midpoint_2.x, slalom_midpoint_2.y),
#     make_waypoint(slalom_midpoint_3.x, slalom_midpoint_3.y),

#     # Exit Slalom
#     make_waypoint(slalom_midpoint_3.x,
#                   slalom_midpoint_3.y + ROBOT_LENGTH + 0.2),

#     # Octagon
#     make_waypoint(octagon.x, octagon.y),
#     make_waypoint(octagon.x, octagon.y, depth=0.0),  # Surface

#     # Return Home
#     make_waypoint(gate_right.x - (ROBOT_WIDTH / 2.0) -
#                   0.1, gate_right.y - ROBOT_LENGTH - 0.25),
# ]
