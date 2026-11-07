"""On the station table: an instrument tray (a steel-walled tray) in the middle of the far side holding a clamp and a
marker, and a green surgical drape on the right, over the patient, with a pair of scissors and a spring clamp still
lying on it. The procedure is over, and every instrument must be back on the tray before the count is signed off:
an instrument left on the drape is a retained surgical item, a "never event". The tray and the drape are fixed; the
four instruments are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "the procedure is finished: return all four instruments to the instrument tray so the count is complete" \\
           --scorer task_score ...
"""

import numpy as np

from yam_arena.demo import pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Positions on the table from the midpoint between the arm bases: x forward, y to the left (m).
# -- the instrument tray (fixed): four low steel walls, the table is its floor ------------------------------------
TRAY_XY, TRAY_INNER = (0.30, 0.0), (0.18, 0.34)  # inside, front to back and left to right
WALL_T, WALL_H = 0.01, 0.015
STEEL = (0.62, 0.64, 0.67)
_ox, _oy = TRAY_INNER[0] / 2 + WALL_T / 2, TRAY_INNER[1] / 2 + WALL_T / 2
TRAY = [
    Box("tray_front", pos=(TRAY_XY[0] - _ox, TRAY_XY[1], TOP + WALL_H / 2),
        size=(WALL_T, TRAY_INNER[1] + 2 * WALL_T, WALL_H), color=STEEL),
    Box("tray_back", pos=(TRAY_XY[0] + _ox, TRAY_XY[1], TOP + WALL_H / 2),
        size=(WALL_T, TRAY_INNER[1] + 2 * WALL_T, WALL_H), color=STEEL),
    Box("tray_left", pos=(TRAY_XY[0], TRAY_XY[1] + _oy, TOP + WALL_H / 2), size=(TRAY_INNER[0], WALL_T, WALL_H),
        color=STEEL),
    Box("tray_right", pos=(TRAY_XY[0], TRAY_XY[1] - _oy, TOP + WALL_H / 2), size=(TRAY_INNER[0], WALL_T, WALL_H),
        color=STEEL),
]

# -- the drape (fixed): a thin green sheet over the patient ---------------------------------------------------------
DRAPE_XY, DRAPE_SIZE = (0.19, -0.275), (0.18, 0.16)
DRAPE = Box("drape", pos=(*DRAPE_XY, TOP + 0.0004), size=(*DRAPE_SIZE, 0.0008), color=(0.25, 0.55, 0.45))

# -- the instruments (loose): library models standing in for surgical instruments ---------------------------------
START = {"scissors": (0.19, -0.235), "spring_clamp": (0.19, -0.32), "clamp": (0.27, 0.09), "marker": (0.35, 0.09)}
INSTRUMENTS = [
    Asset("scissors", "scissors_ycb_robolab", xy=START["scissors"], extent=0.14, mass=0.06, color=(0.7, 0.72, 0.75),
          yaw=np.pi / 2, convex=True),  # 14 x 6.7 cm, lying front to back
    Asset("spring_clamp", "spring_clamp_ycb_robolab", xy=START["spring_clamp"], extent=0.08, mass=0.04,
          color=(0.7, 0.72, 0.75), convex=True),  # 8 x 6.2 cm
    Asset("clamp", "clamp_ycb_robolab", xy=START["clamp"], extent=0.10, mass=0.08, color=(0.7, 0.72, 0.75),
          convex=True),  # 7 x 10 cm
    Asset("marker", "dry_erase_marker_ycb_robolab", xy=START["marker"], extent=0.12, mass=0.02, convex=True),
]
NAMES = [i.name for i in INSTRUMENTS]

OBJECTS = [*TRAY, DRAPE, *INSTRUMENTS]

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
ON_TRAY_DZ = 0.03  # an instrument's underside within 3 cm of the table, inside the tray's walls
ON_DRAPE_DZ = 0.05  # anywhere within 5 cm above the drape counts as still on it (resting or not)


def _inside(item, xy, size) -> bool:
    return abs(item.pos[0] - xy[0]) <= size[0] / 2 and abs(item.pos[1] - xy[1]) <= size[1] / 2


def on_tray(objects, name) -> bool:
    item = objects[name]
    return bool(_inside(item, TRAY_XY, TRAY_INNER) and item.low[2] <= TOP + ON_TRAY_DZ and still(objects, name))


def on_drape(objects, name) -> bool:
    item = objects[name]
    return bool(_inside(item, DRAPE_XY, DRAPE_SIZE) and item.low[2] <= TOP + ON_DRAPE_DZ)


def outcome(objects) -> str:
    """'retained' (any instrument still on the drape: a never event), 'count_correct' (all four on the tray: success),
    'one_missing' (drape clear, three on the tray), or 'drape_clear_short' (drape clear, two or fewer on the tray)."""
    if any(on_drape(objects, n) for n in NAMES):
        return "retained"
    n = sum(on_tray(objects, x) for x in NAMES)
    return {4: "count_correct", 3: "one_missing"}.get(n, "drape_clear_short")


def success(objects):
    """All four instruments rest on the tray, none on the drape."""
    return outcome(objects) == "count_correct"


def distance(objects):
    """Metres (horizontal), summed over the two instruments that start on the drape, to the tray's centre."""
    return float(sum(np.hypot(*(np.asarray(objects[n].pos[:2]) - np.asarray(TRAY_XY)))
                     for n in ("scissors", "spring_clamp")))


def score(objects):
    """0 if any instrument is still on the drape; otherwise 1.0 for all four on the tray, 0.5 for three, 0.25 for
    fewer."""
    return {"count_correct": 1.0, "one_missing": 0.5, "drape_clear_short": 0.25}.get(outcome(objects), 0.0)


# -- scripted demo ------------------------------------------------------------------------------------------------
GRIP_Z = TOP + 0.009  # the instruments are 1.2-1.4 cm thick; the pad tips stop just above the table
DROP_Z = GRIP_Z + 0.02  # let go 2 cm up, over the 1.5 cm walls
TRAVEL_Z = TOP + 0.13
SPOT = {"scissors": (0.26, -0.085), "spring_clamp": (0.345, -0.085)}


def demo():
    """The right arm lifts the scissors off the drape, turns them a quarter turn so they lie left to right, and sets
    them on the tray's free right half; then it does the same with the spring clamp, without turning it. The left
    arm holds its ready pose."""
    scissors = pick_place_waypoints((*START["scissors"], GRIP_Z), (*SPOT["scissors"], DROP_Z), travel_z=TRAVEL_Z,
                                    yaw=0.0, place_yaw=np.pi / 2)
    clamp = pick_place_waypoints((*START["spring_clamp"], GRIP_Z), (*SPOT["spring_clamp"], DROP_Z), travel_z=TRAVEL_Z)
    return {"right": scissors + clamp}
