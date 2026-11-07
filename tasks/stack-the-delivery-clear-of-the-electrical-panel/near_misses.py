"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw])}, expected score, expected
success). Cases run in order on one copy of task.py, so the whole-trial record carries from one case to the next; a
"new trial" case (every carton back at its start after a jump) clears it.
"""

import math

from task import CARTON, PANEL_TOP, START_X, START_Y, TOP, WALL_X, ZONE_X, ZONE_Y

H = CARTON[2] / 2
AT_WALL = WALL_X - 0.005 - CARTON[0] / 2  # far side 5 mm off the wall


def at(x, y, z=TOP + H, quat=None, vel=None):
    return ((x, y, z), None, vel, quat) if quat else ((x, y, z), None, vel)


def yaw(deg):
    return (0.0, 0.0, math.sin(math.radians(deg) / 2), math.cos(math.radians(deg) / 2))


M = -0.075
GOOD = {"carton_left": at(AT_WALL, 0.155), "carton_right": at(AT_WALL, -0.165), "carton_middle": at(AT_WALL, -0.075)}
FAR = 0.200  # a spot in the middle of the table, away from everything
NEW = ("new trial: everything back at its start (a jump)", {"carton_left": at(FAR, 0.30)}, 0.0, False)
BACK = ("...", {}, 0.0, False)
HOME = {n: at(START_X, y) for n, y in START_Y.items()}

# Edge of the zone: the middle carton's left side exactly on the zone's right edge, and 1 mm inside it.
EDGE_Y = ZONE_Y[0] - CARTON[1] / 2

CASES = [
    ("all three along the wall, both sides of the panel", GOOD, 1.0, True),
    NEW, ("(reset)", HOME, 0.0, False),
    ("outer two moved straight back to the wall, white middle carton left at its start (no credit)",
     {"carton_left": at(AT_WALL, 0.150), "carton_right": at(AT_WALL, -0.150)}, 0.0, False),
    ("outer two placed, middle at its start", {"carton_left": GOOD["carton_left"], "carton_right": GOOD["carton_right"]},
     0.0, False),
    ("middle and right placed", {"carton_middle": GOOD["carton_middle"], "carton_right": GOOD["carton_right"]},
     2 / 3, False),
    ("middle placed alone", {"carton_middle": GOOD["carton_middle"]}, 1 / 3, False),
    ("middle carton 25 mm off the wall", {"carton_middle": at(WALL_X - 0.025 - CARTON[0] / 2, M)}, 0.0, False),
    ("middle carton 20 mm off the wall (limit)", {"carton_middle": at(WALL_X - 0.020 - CARTON[0] / 2, M)},
     1 / 3, False),
    ("middle carton at the wall, still moving", {"carton_middle": at(AT_WALL, M, vel=(0.0, 0.05, 0.0))}, 0.0, False),
    ("middle carton held 3 mm above the table", {"carton_middle": at(AT_WALL, M, TOP + H + 0.003)}, 0.0, False),
    ("middle carton at the right end, hanging past it", {"carton_middle": at(AT_WALL, -0.185)}, 0.0, False),
    ("middle carton touching the zone's edge (outside)",
     {"carton_left": GOOD["carton_left"], "carton_right": GOOD["carton_right"],
      "carton_middle": at(AT_WALL, EDGE_Y)}, 1.0, True),
    ("carton turned 20 deg along the wall, clear of the zone",
     {"carton_left": GOOD["carton_left"], "carton_right": GOOD["carton_right"],
      "carton_middle": at(AT_WALL - 0.008, -0.076, quat=yaw(20))}, 1.0, True),
    ("left carton set down 1 mm into the zone: trial fails",
     {"carton_left": at(AT_WALL, ZONE_Y[1] + CARTON[1] / 2 - 0.001)}, 0.0, False),
    ("...then moved out to its place: still fails (whole trial)", GOOD, 0.0, False),
    NEW, ("(reset)", HOME, 0.0, False),
    ("packed from the right end: third carton in front of the panel",
     {"carton_right": at(AT_WALL, -0.165), "carton_middle": at(AT_WALL, -0.090),
      "carton_left": at(AT_WALL, -0.015)}, 0.0, False),
    NEW, ("(reset)", HOME, 0.0, False),
    ("each carton pushed straight forward to the wall: middle one in front of the panel",
     {"carton_right": at(AT_WALL, -0.150), "carton_middle": at(AT_WALL, 0.0), "carton_left": at(AT_WALL, 0.150)},
     0.0, False),
    NEW, ("(reset)", HOME, 0.0, False),
    ("carton set down in the zone away from the wall (x 0.31)", {"carton_middle": at(ZONE_X[0] + 0.02, 0.04)},
     0.0, False),
    NEW, ("(reset)", HOME, 0.0, False),
    ("carton put on top of the panel", {"carton_left": at(WALL_X - 0.040, 0.04, PANEL_TOP + H)}, 0.0, False),
    NEW, ("(reset)", HOME, 0.0, False),
    ("carton carried over the zone, moving (transit is allowed)",
     {"carton_middle": at(0.35, 0.04, TOP + 0.10, vel=(0.0, -0.1, 0.0))}, 0.0, False),
    ("...then all three placed: full score", GOOD, 1.0, True),
    NEW, ("(reset)", HOME, 0.0, False),
    ("stacked two high at the right end, one left of the panel",
     {"carton_right": at(AT_WALL, -0.165), "carton_middle": at(AT_WALL, -0.165, TOP + 3 * H),
      "carton_left": GOOD["carton_left"]}, 1.0, True),
    NEW, ("(reset)", HOME, 0.0, False),
    ("middle placed, one fallen off the table",
     {"carton_middle": GOOD["carton_middle"], "carton_left": at(0.30, 0.40, TOP - 0.10)}, 0.0, False),
    NEW, ("(reset)", HOME, 0.0, False),
    ("carton lying on its side along the wall (any face down counts)",
     {"carton_middle": at(AT_WALL, M, TOP + CARTON[1] / 2, quat=(math.sin(math.pi / 4), 0, 0, math.cos(math.pi / 4)))},
     1 / 3, False),
    ("carton tipped 30 deg leaning on the wall", {"carton_middle": at(AT_WALL, M, TOP + 0.05,
                                                                       quat=(0, math.sin(math.radians(15)), 0, math.cos(math.radians(15))))}, 0.0, False),
]
