"""On the station table: four oranges in the middle of the table, a red tray (Maya's) in front of the left arm and a
blue tray (Leo's) in front of the right arm. Leo's tray already holds two oranges. The robot is asked to share the
oranges fairly between the two children. A fair share counts what Leo already has: three oranges to Maya and one to
Leo, so each ends with three. Splitting the four evenly (two each) leaves Maya with two and Leo with four. The trays
are fixed; the oranges are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "share the oranges fairly between Maya's red tray and Leo's blue tray" \\
           --scorer task_score ...
"""

import numpy as np

from yam_arena.demo import FINGER_OPEN, Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Positions on the table from the midpoint between the arm bases: x forward, y to the left (m).
# -- the trays (fixed): four walls each, the table is their floor ------------------------------------------------
TRAY_INNER = 0.16  # inside, front to back and left to right
WALL_T, WALL_H = 0.01, 0.03
MAYA_XY, LEO_XY = (0.30, 0.22), (0.30, -0.22)
RED, BLUE = (0.8, 0.15, 0.15), (0.15, 0.3, 0.8)


def _tray(owner, xy, color):
    x, y = xy
    off = TRAY_INNER / 2 + WALL_T / 2
    long = TRAY_INNER + 2 * WALL_T
    z = TOP + WALL_H / 2
    return [
        Box(f"{owner}_tray_front", pos=(x - off, y, z), size=(WALL_T, long, WALL_H), color=color),
        Box(f"{owner}_tray_back", pos=(x + off, y, z), size=(WALL_T, long, WALL_H), color=color),
        Box(f"{owner}_tray_left", pos=(x, y + off, z), size=(TRAY_INNER, WALL_T, WALL_H), color=color),
        Box(f"{owner}_tray_right", pos=(x, y - off, z), size=(TRAY_INNER, WALL_T, WALL_H), color=color),
    ]


TRAYS = _tray("maya", MAYA_XY, RED) + _tray("leo", LEO_XY, BLUE)

# -- the oranges (loose): the station's own orange model, at the station's size --------------------------------
ORANGE_EXTENT, ORANGE_MASS = 0.064, 0.11
ON_TABLE = {"orange_1": (0.16, 0.06), "orange_2": (0.26, 0.06), "orange_3": (0.26, -0.06), "orange_4": (0.16, -0.06)}
LEOS_HEAD_START = {"leo_1": (LEO_XY[0] - 0.04, LEO_XY[1] + 0.02), "leo_2": (LEO_XY[0] + 0.04, LEO_XY[1] + 0.02)}
ORANGES = [Asset(n, "orange_02_fruits_veggies_robolab", xy=xy, extent=ORANGE_EXTENT, mass=ORANGE_MASS)
           for n, xy in {**ON_TABLE, **LEOS_HEAD_START}.items()]
ORANGE_NAMES = [o.name for o in ORANGES]

OBJECTS = [*TRAYS, *ORANGES]

# -- stillness ----------------------------------------------------------------------------------------------------
_STEP = {"objects": None, "prev": {}, "cur": {}}


def still(objects, name) -> bool:
    """At rest: yam_arena's resting() (under 1 cm/s and 0.5 rad/s), or not moved at all since the previous control
    step (under 0.5 mm, and under 0.01 in each quaternion component). The second test is needed because PhysX can
    put a light, settled body to sleep while it still reports the small velocity it had just before, so resting()
    would never become true for it. The hooks run once per control step on the same ``objects``, so a new
    ``objects`` dict marks a new step."""
    if objects is not _STEP["objects"]:
        _STEP["objects"], _STEP["prev"] = objects, _STEP["cur"]
        _STEP["cur"] = {n: np.r_[np.asarray(o.pos, dtype=float), np.asarray(o.quat_xyzw, dtype=float)]
                        for n, o in objects.items()}
    now, last = _STEP["cur"][name], _STEP["prev"].get(name)
    unmoved = (last is not None and float(np.linalg.norm(now[:3] - last[:3])) < 0.0005
               and float(np.abs(now[3:] - last[3:]).max()) < 0.01)
    return bool(objects[name].resting() or unmoved)


# -- scoring ------------------------------------------------------------------------------------------------------
IN_TRAY_DZ = 0.035  # an orange's underside within 3.5 cm of the table (so one resting on another still counts)


def in_tray(objects, name, tray_xy) -> bool:
    """The orange's centre is inside the tray's walls, its underside near the table, and it is still."""
    orange = objects[name]
    dx, dy = np.abs(np.asarray(orange.pos[:2]) - np.asarray(tray_xy))
    inside = dx <= TRAY_INNER / 2 and dy <= TRAY_INNER / 2
    return bool(inside and orange.low[2] <= TOP + IN_TRAY_DZ and still(objects, name))


def counts(objects):
    """(oranges in Maya's red tray, oranges in Leo's blue tray)."""
    maya = sum(in_tray(objects, n, MAYA_XY) for n in ORANGE_NAMES)
    leo = sum(in_tray(objects, n, LEO_XY) for n in ORANGE_NAMES)
    return maya, leo


def outcome(objects) -> str:
    """'fair' (three each: success), 'one_off' (one tray has one more), 'equal_short' (equal, but some oranges were
    not shared out), or 'unfair' (two or more apart, including the untouched start, nil to two)."""
    maya, leo = counts(objects)
    if maya == leo == len(ORANGE_NAMES) // 2:
        return "fair"
    if abs(maya - leo) == 1:
        return "one_off"
    if maya == leo:
        return "equal_short"
    return "unfair"


def success(objects):
    """Three oranges rest in each tray."""
    return outcome(objects) == "fair"


def distance(objects):
    """How many oranges the two trays differ by."""
    maya, leo = counts(objects)
    return float(abs(maya - leo))


def score(objects):
    """Partial credit (success scores 1.0): 0.5 one orange apart; 0.25 equal but with oranges left over; 0 two or
    more apart."""
    return {"fair": 1.0, "one_off": 0.5, "equal_short": 0.25}.get(outcome(objects), 0.0)


# -- scripted demo ------------------------------------------------------------------------------------------------
GRIP_Z = TOP + 0.030  # the orange's middle (the station's own grip height for it)
DROP_Z = TOP + 0.095  # let go with the orange's underside 6.5 cm up, clear of the 3 cm walls and of oranges inside
TRAVEL_Z = TOP + 0.16
MAYA_SPOTS = [(MAYA_XY[0] - 0.04, MAYA_XY[1] - 0.02), (MAYA_XY[0] + 0.04, MAYA_XY[1] - 0.02),
              (MAYA_XY[0], MAYA_XY[1] + 0.045)]
LEO_SPOT = (LEO_XY[0], LEO_XY[1] - 0.045)


def demo():
    """The right arm moves one orange to Leo's blue tray, which already has two. The left arm waits above Maya's tray
    until the right arm is done (the two grippers would meet in the middle of the table), then moves the other
    three oranges to Maya's red tray, one at a time. Each tray ends with three."""
    right = pick_place_waypoints((*ON_TABLE["orange_3"], GRIP_Z), (*LEO_SPOT, DROP_Z), travel_z=TRAVEL_Z)
    wait = Waypoint(sum(w.duration for w in right), (MAYA_XY[0] - 0.08, MAYA_XY[1], TRAVEL_Z), FINGER_OPEN,
                    "wait for the right arm")
    left = [wait] + [w for n, spot in zip(["orange_2", "orange_1", "orange_4"], MAYA_SPOTS)
                     for w in pick_place_waypoints((*ON_TABLE[n], GRIP_Z), (*spot, DROP_Z), travel_z=TRAVEL_Z)]
    return {"left": left, "right": right}
