"""Two-arm carry via protective edge clamps; never move the board directly."""
import os
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN
import geometry as g


def _arm(side):
    sign = 1 if side == "left" else -1
    y0 = sign * .079
    # The revised fixture's first Isaac run released the board 0.55 mm past
    # the X target and 0.12 mm toward +Y. Compensate at the arm target.
    y1 = y0 + .00178
    x0, _, z0 = g.BOARD_START
    x1, _, z1 = g.BOARD_GOAL
    grip_x = -.060 if side == "left" else +.050
    x0 += grip_x
    x1 += grip_x + .00545
    # Pad tips sit 13 mm below TCP and the clamp's centre is 10 mm above PCB.
    pick = (x0, y0, z0 + .023)
    place = (x1, y1, z1 + .023)
    travel = g.TOP + .120
    grip = -.010  # 20 mm finger gap around each 26 mm clamp
    # The first two Isaac runs carried the +X edge about 5 degrees low.
    # Split the correction across both arms to stay inside their reach.
    level_z = .001 if side == "right" else -.007
    return [
        Waypoint(1.6, (x0, y0, travel), FINGER_OPEN, "clamp approach"),
        Waypoint(1.5, pick, FINGER_OPEN, "clamp descend"),
        Waypoint(.8, pick, grip, "grip edge clamp"),
        Waypoint(1.4, (x0, y0, travel+level_z), grip, "board lift"),
        Waypoint(2.2, (x1, y1, travel+level_z), grip, "board carry"),
        Waypoint(1.4, (x1, y1, place[2]+.012+level_z), grip, "align above standoffs"),
        Waypoint(2.0, (x1, y1, place[2]-.002+level_z), grip, "lower onto standoffs"),
        Waypoint(1.2, (x1, y1, place[2]-.002+level_z), grip, "hold seated board"),
        Waypoint(1.0, (x1, y1, place[2]-.002+level_z), FINGER_OPEN, "release clamp"),
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
