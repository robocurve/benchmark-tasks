"""Plain-Python checks of score() and success() in task.py; no simulator needed.

    python near_misses.py
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import task  # noqa: E402
from labkit import PartState  # noqa: E402

Z = np.zeros(3)
TX, TY, TZ = task.TRAY


def unit(drawer):
    return PartState(pos=Z, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=Z, ang_vel=Z, low=Z, high=Z,
                     joints={"drawer": drawer})


def tray(x, y, z, pitch_deg=0.0, moving=False):
    """The tray's body origin (middle of its underside) at (x, y, z), tipped about y by pitch_deg."""
    a = math.radians(pitch_deg)
    q = np.array([0.0, math.sin(a / 2), 0.0, math.cos(a / 2)])
    hx = TX / 2 * math.cos(a) + TZ / 2 * abs(math.sin(a))
    lo = np.array([x - hx, y - TY / 2, z - TX / 2 * abs(math.sin(a))])
    hi = np.array([x + hx, y + TY / 2, z + TZ])
    return PartState(pos=np.array([x, y, z]), quat_xyzw=q, lin_vel=np.array([0.1, 0, 0]) if moving else Z,
                     ang_vel=Z, low=lo, high=hi)


def mid(drawer):
    return sum(task.IN_X) / 2 + drawer


def objs(drawer, organizer):
    return {"unit": unit(drawer), "organizer": organizer, "pen": tray(0.2, -0.23, task.TOP)}


F = task.FLOOR_Z
cases = [
    ("untouched: tray on the table, drawer half out", objs(task.START, tray(*task.TRAY_START, task.TOP)), 0.0),
    ("drawer shut, tray left on the table", objs(0.0, tray(*task.TRAY_START, task.TOP)), 0.0),
    ("seated flat, drawer shut", objs(0.0, tray(mid(0.0), 0.0, F)), 1.0),
    ("seated flat, drawer 4 mm from shut", objs(-0.004, tray(mid(-0.004), 0.0, F)), 1.0),
    ("seated flat, drawer 8 mm from shut", objs(-0.008, tray(mid(-0.008), 0.0, F)), 0.5),
    ("seated flat, drawer still pulled out", objs(-task.TRAVEL, tray(mid(-task.TRAVEL), 0.0, F)), 0.5),
    ("riding on the pen (14 mm up at the back), drawer out",
     objs(-task.TRAVEL, tray(mid(-task.TRAVEL), 0.0, F + 0.004, pitch_deg=-3.0)), 0.25),
    ("lying level on top of the pen, 14 mm up", objs(-task.TRAVEL, tray(mid(-task.TRAVEL), 0.0, F + 0.014)), 0.25),
    ("in the drawer but tipped up on edge", objs(-task.TRAVEL, tray(mid(-task.TRAVEL), 0.0, F + 0.06, pitch_deg=80)),
     0.25),
    ("half over the drawer's front wall", objs(-task.TRAVEL, tray(mid(-task.TRAVEL) - 0.12, 0.0, F)), 0.0),
    ("held above the drawer", objs(-task.TRAVEL, tray(mid(-task.TRAVEL), 0.0, F + 0.08)), 0.0),
    ("on top of the unit, drawer shut", objs(0.0, tray(0.476, 0.0, task.TOP + task.OPEN_H + task.TOP_T)), 0.0),
]
ok = True
for label, o, want in cases:
    got = task.score(o)
    good = abs(got - want) < 1e-9
    ok &= good
    print(f"{'ok ' if good else 'BAD'}  {got:.2f} (want {want:.2f})  {label}")

o = objs(0.0, tray(mid(0.0), 0.0, F))
good = task.success(o)
o["organizer"] = tray(mid(0.0), 0.0, F, moving=True)
good &= not task.success(o)
ok &= good
print(f"{'ok ' if good else 'BAD'}  success() needs 1.0 and the tray at rest")

# the pen: three places along the drawer's back, chosen by seed, every one under a seated tray's footprint
spots = {task.pen_spot(s) for s in range(100)}
good = spots == set(task.PEN_YS) and task.pen_spot(7) == task.pen_spot(7)
ok &= good
print(f"{'ok ' if good else 'BAD'}  pen_spot(): all three places over seeds 0-99; the same seed gives the same place")
# a tray riding on the pen is too tall to pass under the unit's top
good = 2 * task.PEN_R + TZ > task.OPEN_H - (task.FLOOR_Z - task.TOP) and TZ < task.OPEN_H - (task.FLOOR_Z - task.TOP)
ok &= good
print(f"{'ok ' if good else 'BAD'}  on the pen the tray stands {(2 * task.PEN_R + TZ) * 1000:.0f} mm, over the "
      f"{(task.OPEN_H - (task.FLOOR_Z - task.TOP)) * 1000:.0f} mm under the unit's top; flat it is {TZ * 1000:.0f} mm")
print("all checks passed" if ok else "SOME CHECKS FAILED")
sys.exit(0 if ok else 1)
