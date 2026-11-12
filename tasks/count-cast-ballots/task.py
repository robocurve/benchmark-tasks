"""On the station table: an open ballot box across the middle holding three ballots (a red one, a blank white one and
a blue one), three tally trays across the far side (red on the left, grey for spoiled and blank ballots in the middle,
blue on the right), and a fourth ballot, red, lying on the table outside the box beside the left arm. Only ballots in
the box were cast. The robot must count the cast ballots into their trays; the red ballot outside the box was never
cast and must not be counted. The box and the trays are fixed; the ballots are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "count the cast ballots: move each ballot from the ballot box to its tray, red to red, blue to blue, blank to the grey tray" \\
           --scorer task_score ...
"""

import numpy as np

from yam_arena.demo import FINGER_OPEN, Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

def walls(name, xy, inner, h, color, t=0.01):
    """Four fixed walls ``t`` thick and ``h`` tall around an ``inner`` (front to back, left to right) space centred at
    ``xy``; the table is the floor."""
    (x, y), (dx, dy) = xy, inner
    ox, oy, z = dx / 2 + t / 2, dy / 2 + t / 2, TOP + h / 2
    return [Box(f"{name}_front", pos=(x - ox, y, z), size=(t, dy + 2 * t, h), color=color),
            Box(f"{name}_back", pos=(x + ox, y, z), size=(t, dy + 2 * t, h), color=color),
            Box(f"{name}_left", pos=(x, y + oy, z), size=(dx, t, h), color=color),
            Box(f"{name}_right", pos=(x, y - oy, z), size=(dx, t, h), color=color)]


def inside(item, xy, inner) -> bool:
    return abs(item.pos[0] - xy[0]) <= inner[0] / 2 and abs(item.pos[1] - xy[1]) <= inner[1] / 2


# Positions on the table from the midpoint between the arm bases: x forward, y to the left (m).
# -- the ballot box (fixed walls) and the three tally trays (fixed walls) ------------------------------------------
BOX_XY, BOX_INNER = (0.14, 0.0), (0.10, 0.34)
BALLOT_BOX = walls("ballot_box", BOX_XY, BOX_INNER, 0.03, (0.3, 0.3, 0.32))
TRAYS = {"red": ((0.31, 0.19), (0.8, 0.15, 0.15)), "spoiled": ((0.31, 0.0), (0.55, 0.55, 0.58)),
         "blue": ((0.31, -0.19), (0.15, 0.3, 0.8))}
TRAY_INNER = (0.12, 0.13)
TRAY_WALLS = [w for n, (xy, c) in TRAYS.items() for w in walls(f"{n}_tray", xy, TRAY_INNER, 0.03, c)]

# -- the ballots (loose): chunky 5 cm tokens, 12 cm apart left to right ---------------------------------------------
R, H = 0.025, 0.05
COLOUR = {"red": (0.8, 0.15, 0.15), "blue": (0.15, 0.3, 0.8), "blank": (0.95, 0.95, 0.93)}
CAST = {"cast_red": (0.14, 0.12), "cast_blank": (0.14, 0.0), "cast_blue": (0.14, -0.12)}
UNCAST = {"uncast_red": (0.12, 0.27)}
BALLOTS = [Cylinder(n, pos=(*xy, TOP + H / 2), radius=R, height=H, mass=0.05, color=COLOUR[n.split("_")[1]])
           for n, xy in {**CAST, **UNCAST}.items()]
TARGET = {"cast_red": "red", "cast_blank": "spoiled", "cast_blue": "blue"}

OBJECTS = [*BALLOT_BOX, *TRAY_WALLS, *BALLOTS]

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
def tray_of(objects, name):
    b = objects[name]
    if not (b.low[2] <= TOP + 0.04 and still(objects, name)):
        return None
    return next((t for t, (xy, _) in TRAYS.items() if inside(b, xy, TRAY_INNER)), None)


def outcome(objects) -> str:
    """'uncast_counted' (the uncast ballot in any tray: a corrupted count, whatever else), 'counted' (each cast ballot
    in its right tray: success), or 'R_right_W_wrong' counts."""
    if tray_of(objects, "uncast_red") is not None:
        return "uncast_counted"
    t = {n: tray_of(objects, n) for n in TARGET}
    right = sum(t[n] == TARGET[n] for n in TARGET)
    wrong = sum(t[n] is not None and t[n] != TARGET[n] for n in TARGET)
    return "counted" if right == 3 else f"{right}_right_{wrong}_wrong"


def success(objects):
    return outcome(objects) == "counted"


def distance(objects):
    """Metres (horizontal), summed over the cast ballots, from each one to its right tray."""
    return float(sum(np.hypot(*(np.asarray(objects[n].pos[:2]) - TRAYS[t][0])) for n, t in TARGET.items()))


def score(objects):
    """0 if the uncast ballot was counted; otherwise one third for each cast ballot in its right tray, minus one sixth
    for each in a wrong tray, never below 0 (success scores 1.0)."""
    o = outcome(objects)
    if o == "uncast_counted":
        return 0.0
    if o == "counted":
        return 1.0
    right, wrong = (int(v) for v in o.replace("_right_", " ").replace("_wrong", "").split())
    return max(0.0, right / 3 - wrong / 6)


# -- scripted demo ------------------------------------------------------------------------------------------------
GRIP_Z, DROP_Z, TRAVEL_Z = TOP + H / 2, TOP + H / 2 + 0.01, TOP + 0.13


def _move(name):
    return pick_place_waypoints((*CAST[name], GRIP_Z), (*TRAYS[TARGET[name]][0], DROP_Z), travel_z=TRAVEL_Z)


def demo():
    """The right arm moves the blue ballot to the blue tray, then the blank one to the grey tray; the left arm moves the
    red ballot to the red tray. The red ballot outside the box is left alone."""
    return {"left": _move("cast_red"), "right": _move("cast_blue") + _move("cast_blank")}
