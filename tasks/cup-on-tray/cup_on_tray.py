"""On the station table: a dark takeaway coffee cup with a lid (8 cm across, 12.6 cm tall, standing upright) in front of the left arm,
and a grey tray (20 x 16 x 2 cm board) between the arms, a little further out. Both are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=cup_on_tray.py \\
           --instruction "put the cup on the tray" --scorer task_score ...
"""

import numpy as np

from yam_arena.demo import pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

CUP_MODEL = "gregorys_coffee_cup_objaverse_robolab"  # fallback: "mug_ycb_robolab" (has a handle)
CUP_XY = (0.17, 0.12)
CUP_W = 0.08  # widest horizontal extent (m)
GRASP_H = 0.06  # grasp this far above the cup's base (m); keep below the cup's height
TRAY_SIZE = (0.20, 0.16, 0.02)

CUP = Asset("cup", asset=CUP_MODEL, xy=CUP_XY, extent=CUP_W, mass=0.05, convex=True)
TRAY = Box("tray", pos=(0.25, -0.05, TOP + TRAY_SIZE[2] / 2), size=TRAY_SIZE, mass=0.3, color=(0.45, 0.45, 0.48))
OBJECTS = [CUP, TRAY]

MAX_TILT_DEG = 10.0  # "upright": the cup's axis within 10 degrees of where it stood at the start

_UP_LOCAL = {}  # the cup's local axis that pointed up at the start (the model's own up axis may not be +z)


def _rot(quat_xyzw):
    x, y, z, w = quat_xyzw
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt_deg(state):
    """Angle between the cup's starting up axis, as it points now, and world vertical (degrees)."""
    rot = _rot(state.quat_xyzw)
    if "cup" not in _UP_LOCAL:  # the first step of a trial: the cup is standing as placed
        _UP_LOCAL["cup"] = rot.T @ np.array([0.0, 0.0, 1.0])
    up_z = float((rot @ _UP_LOCAL["cup"])[2])
    return float(np.degrees(np.arccos(np.clip(up_z, -1.0, 1.0))))


def _on_tray(objects, margin=0.0):
    """The cup's centre is over the tray (shrunk by ``margin`` on every side) and its bottom is on the tray top."""
    cup, tray = objects["cup"], objects["tray"]
    over = (
        tray.low[0] + margin < cup.pos[0] < tray.high[0] - margin
        and tray.low[1] + margin < cup.pos[1] < tray.high[1] - margin
    )
    return bool(over and abs(cup.low[2] - tray.high[2]) < 0.01)


def success(objects):
    """Upright on the tray with its whole base on it (centre at least half a cup width in from every edge),
    both still."""
    tilt = _tilt_deg(objects["cup"])
    return bool(
        _on_tray(objects, margin=CUP_W / 2)
        and tilt < MAX_TILT_DEG
        and objects["cup"].resting()
        and objects["tray"].resting()
    )


def distance(objects):
    cup, tray = objects["cup"], objects["tray"]
    half_h = (cup.high[2] - cup.low[2]) / 2
    goal = np.array([tray.pos[0], tray.pos[1], tray.high[2] + half_h])
    return float(np.linalg.norm(cup.pos - goal))


def score(objects):
    """Partial credit (success scores 1.0): 0.5 if the cup is on the tray but tipped over or overhanging an edge."""
    _tilt_deg(objects["cup"])  # records the starting up axis even if score() runs before success()
    return 0.5 if _on_tray(objects) else 0.0


def demo():
    """Left arm, at half speed: grasp the cup GRASP_H above its base, carry it 14 cm above the table, release it 1 cm
    above the tray's centre."""
    pick = (*CUP_XY, TOP + GRASP_H)
    place = (TRAY.pos[0], TRAY.pos[1], TOP + TRAY_SIZE[2] + GRASP_H + 0.01)
    return {"left": pick_place_waypoints(pick, place, travel_z=TOP + 0.14, speed=0.5)}
