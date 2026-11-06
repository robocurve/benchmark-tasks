# SPDX-License-Identifier: MIT
"""Mark a perpendicular cut line on the fixed wooden board.

Two loose tools stand beside a fixed 140 x 280 x 16 mm board: a blue-stock
carpenter's square to the left and an orange 22 mm long-nose marker in a holder to the
right. A small red tick on the board's near long edge locates the cut. Seat the
square's stock against that edge so its blade follows the required cut, hold
it steady, and draw across the board along the blade. Only the black marker
nib deposits ink; the orange barrel does not. Lift the nib after drawing.
All dimensions are metres in the simulator. The board and tool holder are fixed.
"""
from pathlib import Path
import math
from yam_arena.task_file import Box
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED

SCENE = "station"
TABLE_TOP = 0.7475
BOARD_CENTER = (0.29, 0.0, 0.7555)
BOARD_SIZE = (0.14, 0.28, 0.016)
BOARD_TOP = 0.7635
TARGET_START = (0.22, -0.03)
TARGET_END = (0.36, -0.03)
MARKER_NIB_LOCAL = (0.0, 0.0, -0.05)
MARKER_NIB_RADIUS = 0.002
MARKER_GRASP_LOCAL = (0.0, 0.0, 0.023)
SQUARE_BLADE_LOCAL_EDGE = ((0.0, -0.1175, -0.002), (0.17, -0.1175, -0.002))
SQUARE_STOCK_INNER_X = 0.012
REQUIRE_GUIDE_CONTACT = False
INK_WIDTH = 0.002
VISUAL_ONLY = ("reference_tick",)
SUCCESS_HOLD_S = 0.5
SCORE_OVER = "end"
ASSETS = Path(__file__).parent / "assets"
COMPOUND_ASSETS = {name: str(ASSETS / (name + ".usda")) for name in ("board", "marker", "square")}
INSTRUCTION = ("Use the carpenter's square and orange marker to draw one straight cut line "
               "through the red tick, perpendicular to the board's long edge and across its width. "
               "Keep the square seated against the near edge while marking, avoid extra marks, "
               "then lift the marker clear of the board.")

OBJECTS = [
    Box("board", pos=BOARD_CENTER, size=BOARD_SIZE, color=(0.72,0.52,0.30)),
    Box("square", pos=(0.18,0.27,0.7655), size=(0.024,0.25,0.036), mass=0.18),
    Box("marker", pos=(0.18,-0.19,0.8175), size=(0.022,0.022,0.08), mass=0.04),
    Box("reference_tick", pos=(0.222,-0.03,BOARD_TOP+0.00015), size=(0.004,0.002,0.0002), color=(0.9,0.05,0.03)),
    Box("holder_base", pos=(0.18,-0.19,TABLE_TOP+0.009), size=(0.04,0.04,0.018), color=(0.15,0.18,0.22)),
    Box("holder_front", pos=(0.175,-0.19,TABLE_TOP+0.0255), size=(0.004,0.014,0.015), color=(0.15,0.18,0.22)),
    Box("holder_back", pos=(0.185,-0.19,TABLE_TOP+0.0255), size=(0.004,0.014,0.015), color=(0.15,0.18,0.22)),
    Box("holder_left", pos=(0.18,-0.185,TABLE_TOP+0.0255), size=(0.006,0.004,0.015), color=(0.15,0.18,0.22)),
    Box("holder_right", pos=(0.18,-0.195,TABLE_TOP+0.0255), size=(0.006,0.004,0.015), color=(0.15,0.18,0.22)),
]


def success(objects):
    """Scoring is supplied by the contact-aware embodiment, not instantaneous poses."""
    return False


def score(objects):
    return 0.0


def pickup_waypoints():
    """Initial physical tool grasps; subsequent motion uses measured-pose feedback."""
    O, C = FINGER_OPEN, FINGER_CLOSED
    yaw = -math.pi / 2
    return {
        "left": [
            Waypoint(1.5,(0.18,0.37,0.84),O,"square hover",yaw),
            Waypoint(1.5,(0.18,0.37,0.7825),O,"square grasp",yaw),
            Waypoint(0.8,(0.18,0.37,0.7825),C,"grip square",yaw),
            Waypoint(1.2,(0.18,0.37,0.84),C,"lift square",yaw),
        ],
        "right": [
            Waypoint(1.5,(0.18,-0.19,0.875),O,"marker hover"),
            Waypoint(1.5,(0.18,-0.19,0.8405),O,"marker grasp"),
            Waypoint(0.8,(0.18,-0.19,0.8405),C,"grip marker"),
            Waypoint(1.3,(0.18,-0.19,0.875),C,"lift marker"),
        ],
    }
