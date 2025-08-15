from pontus_autonomy.helpers.GoToPoseClient import MovementMethod
from pontus_mapping.semantic_map_manager import SemanticObject
from pontus_autonomy.helpers.run_info_helpers import *

# ----------- ALL CHANGES AFTER THIS POINT ---------------


# ------- Semantic Map Info -------

# ------ Gate Task ------
GATE_CENTER_X = 0.0
GATE_CENTER_Y = 0.0

gate_left = SemanticMapObject(
    label=SemanticObject.LeftGate,
    x=GATE_CENTER_X + 1.5,
    y=GATE_CENTER_Y + 0.0
)

gate_right = SemanticMapObject(
    label=SemanticObject.RightGate,
    x=GATE_CENTER_X - 1.5,
    y=GATE_CENTER_Y + 0.0
)

# ------ Slalom Task -------


# --- Slalom Set 1 ---
RED_SLALOM_1_X = GATE_CENTER_X + 2.0
RED_SLALOM_1_Y = GATE_CENTER_Y + 3.0

slalom_set_1 = SlalomSet(
    red_slalom=SemanticMapObject(
        label=SemanticObject.SlalomRed,
        x=RED_SLALOM_1_X,
        y=RED_SLALOM_1_Y
    ),

    white_slalom_left=SemanticMapObject(
        label=SemanticObject.SlalomWhite,
        x=RED_SLALOM_1_X + 1.5,
        y=RED_SLALOM_1_Y + 0.0
    ),

    white_slalom_right=SemanticMapObject(
        label=SemanticObject.SlalomWhite,
        x=RED_SLALOM_1_X - 1.5,
        y=RED_SLALOM_1_Y + 0.0
    )
)


# --- Slalom Set 2 ---
RED_SLALOM_2_X = RED_SLALOM_1_X + 1.0
RED_SLALOM_2_Y = RED_SLALOM_1_Y + 2.0

slalom_set_2 = SlalomSet(
    red_slalom=SemanticMapObject(
        label=SemanticObject.SlalomRed,
        x=RED_SLALOM_2_X,
        y=RED_SLALOM_2_Y
    ),

    white_slalom_left=SemanticMapObject(
        label=SemanticObject.SlalomWhite,
        x=RED_SLALOM_2_X + 1.5,
        y=RED_SLALOM_2_Y + 0.0
    ),

    white_slalom_right=SemanticMapObject(
        label=SemanticObject.SlalomWhite,
        x=RED_SLALOM_2_X - 1.5,
        y=RED_SLALOM_2_Y + 0.0
    )
)


# --- Slalom Set 3 ---
RED_SLALOM_3_X = RED_SLALOM_2_X + 1.0
RED_SLALOM_3_Y = RED_SLALOM_2_Y + 2.0

slalom_set_3 = SlalomSet(
    red_slalom=SemanticMapObject(
        label=SemanticObject.SlalomRed,
        x=RED_SLALOM_3_X,
        y=RED_SLALOM_3_Y
    ),

    white_slalom_left=SemanticMapObject(
        label=SemanticObject.SlalomWhite,
        x=RED_SLALOM_3_X - 1.5,
        y=RED_SLALOM_3_Y + 0.0
    ),

    white_slalom_right=SemanticMapObject(
        label=SemanticObject.SlalomWhite,
        x=RED_SLALOM_3_X + 1.5,
        y=RED_SLALOM_3_Y + 0.0
    )
)

slalom_midpoint_1: XY = slalom_set_1.get_right_side_midpoint()
slalom_midpoint_2: XY = slalom_set_2.get_right_side_midpoint()
slalom_midpoint_3: XY = slalom_set_3.get_right_side_midpoint()

# ------- Octagon ------
octagon = SemanticMapObject(
    label=SemanticObject.Octagon,
    x=GATE_CENTER_X - 2.5,
    y=GATE_CENTER_Y + 18.0
)

"""
Full semantic map reference:

semantic_map: list[SemanticMapObject] = [
    gate_left,
    gate_right,
    slalom_set_1.red_slalom, slalom_set_1.white_slalom_left, slalom_set_1.white_slalom_right,
    slalom_set_2.red_slalom, slalom_set_2.white_slalom_left, slalom_set_2.white_slalom_right,
    slalom_set_3.red_slalom, slalom_set_3.white_slalom_left, slalom_set_3.white_slalom_right,
    octagon
]

"""

semantic_map: list[SemanticMapObject] = [
    gate_left,
    gate_right,
    slalom_set_1.red_slalom, slalom_set_1.white_slalom_left, slalom_set_1.white_slalom_right,
    slalom_set_2.red_slalom, slalom_set_2.white_slalom_left, slalom_set_2.white_slalom_right,
    slalom_set_3.red_slalom, slalom_set_3.white_slalom_left, slalom_set_3.white_slalom_right,
    octagon
]

# ------- Waypoints ------

# Square Pattern
waypoints_list: list = [
    make_waypoint(0, 0),
    make_waypoint(1, 0),
    make_waypoint(1, 1),
    make_waypoint(1, 0),
    make_waypoint(0, 0, depth=0.0)
]

"""
Full Run Example Reference:

waypoints_list: list[PoseObj] = [
    # Through the gate (slightly right of gate center)
    make_waypoint(gate_right.x - (ROBOT_WIDTH / 2.0) -
                  0.1, gate_right.y + 0.2),

    # Slalom Task
    make_waypoint(slalom_midpoint_1.x, slalom_midpoint_1.y),
    make_waypoint(slalom_midpoint_2.x, slalom_midpoint_2.y),
    make_waypoint(slalom_midpoint_3.x, slalom_midpoint_3.y),

    # Exit Slalom
    make_waypoint(slalom_midpoint_3.x,
                  slalom_midpoint_3.y + ROBOT_LENGTH + 0.2),

    # Octagon
    make_waypoint(octagon.x, octagon.y),
    make_waypoint(octagon.x, octagon.y, depth=0.0),  # Surface

    # Return Home
    make_waypoint(gate_right.x - (ROBOT_WIDTH / 2.0) -
                  0.1, gate_right.y - ROBOT_LENGTH - 0.25),
]

"""
