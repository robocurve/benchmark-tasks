"""Two-arm carry via protective edge clamps; never move the board directly."""
import os
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN
import geometry as g


def _arm(side):
    sign = 1 if side == "left" else -1
    y0 = sign * .079
    # Measured from the completed Isaac run: the open-loop carry released the
    # board 6.3 mm short in X and 1.7 mm toward -Y. Compensate at placement.
    y1 = y0 + .0017
    x0, _, z0 = g.BOARD_START
    x1, _, z1 = g.BOARD_GOAL
    grip_x = -.060 if side == "left" else +.050
    x0 += grip_x
    x1 += grip_x + .0063
    # Pad tips sit 13 mm below TCP and the clamp's centre is 10 mm above PCB.
    pick = (x0, y0, z0 + .023)
    place = (x1, y1, z1 + .023)
    travel = g.TOP + .120
    grip = -.010  # 20 mm finger gap around each 26 mm clamp
    return [
        Waypoint(1.6, (x0, y0, travel), FINGER_OPEN, "clamp approach"),
        Waypoint(1.5, pick, FINGER_OPEN, "clamp descend"),
        Waypoint(.8, pick, grip, "grip edge clamp"),
        Waypoint(1.4, (x0, y0, travel), grip, "board lift"),
        Waypoint(2.2, (x1, y1, travel), grip, "board carry"),
        Waypoint(1.4, (x1, y1, place[2]+.012), grip, "align above standoffs"),
        Waypoint(2.0, (x1, y1, place[2]-.002), grip, "lower onto standoffs"),
        Waypoint(1.2, (x1, y1, place[2]-.002), grip, "hold seated board"),
        Waypoint(1.0, (x1, y1, place[2]-.002), FINGER_OPEN, "release clamp"),
        Waypoint(1.0, (x1, y1, travel), FINGER_OPEN, "raise above board"),
        Waypoint(1.2, (x1, sign*.18, travel), FINGER_OPEN, "withdraw"),
        Waypoint(1.8, g.PARK[side], FINGER_OPEN, "clear"),
    ]


def plans(mode=None):
    mode = mode or os.environ.get("DEMO_MODE", "demo")
    if mode == "demo":
        return {side: _arm(side) for side in ("left", "right")}
    if mode in ("settle", "no_board"):
        return {side: [Waypoint(8, g.PARK[side], FINGER_OPEN, mode)]
                for side in ("left", "right")}
    raise ValueError(mode)
