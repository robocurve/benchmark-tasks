"""Near misses and unintended strategies, each played as a short timeline through the scorer's own step record.

The scorer keeps a per-trial record that only the simulator's step and reset hooks write (hands clear, how long the
scene has been still, whether the bottle ever fell off the lower shelf). Each case below resets that record, plays a
timeline of states at 20 Hz through every loaded copy of the task's step function, and leaves the timeline's last
state for check.py to score. Run alone: python near_misses.py
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import task  # noqa: E402
import lots  # noqa: E402
import parts  # noqa: E402

DT = 0.05
MM = 1e-3
FAR = [np.array([[0.20, 0.30, task.TOP + 0.30]])]  # both hands parked well away
TOP, LZ = task.TOP, task.LZ


def quat_tilt(deg, axis=(0.0, 1.0, 0.0)):
    return tuple(parts._matrix_to_quat(parts._rot(axis, math.radians(deg))))


def scene(slide=0.0, level=0.0, bottle=None, tilt=0.0, vel=None, lot="A"):
    """The fridge with its front half moved back `slide` and up `level` (m), and the bottle's base centre at `bottle`
    (x, y, height above the lower shelf; default: where the lot starts it), leaning `tilt` degrees."""
    L = task.LOTS[lots.index(lot)]
    if bottle is None:
        start = L.pos(task.BOTTLE.name)
        bottle = (start[0], start[1], start[2] - TOP - LZ)
    pos = (bottle[0], bottle[1], TOP + LZ + bottle[2])
    return {task.FRIDGE.name: task.FRIDGE.state(joints={"slide": slide, "level": level}),
            task.BOTTLE.name: task.BOTTLE.state(pos=pos, quat_xyzw=quat_tilt(tilt), lin_vel=vel)}


def bay(lot="A", dx=0.0, dz=0.0005):
    y = 0.060 if lot == "A" else -0.060
    return (task.BAY_X + dx, y, dz)


TUCK = dict(slide=task.TRAVEL, level=-task.DROP)
FULL = scene(**TUCK, bottle=bay())


def near(objects, which, gap=0.003):
    """One gripper point `gap` above the top of the front half (which="shelf") or beside the bottle's body."""
    if which == "shelf":
        boxes = task.FRIDGE.world_boxes({"level": (np.asarray(objects["fridge"].links["level"][0]),
                                                   task._R(objects["fridge"].links["level"][1]))})
        c, r, s = boxes[1][1], boxes[1][2], boxes[1][3]  # the front trim
        top = c[2] + (np.abs(r) @ (s / 2))[2]
        return FAR + [np.array([[c[0], c[1], top + gap]])]
    b = objects["bottle"].pos
    return FAR + [np.array([[b[0] - task.BOTTLE_R - gap, b[1], b[2] + 0.05]])]


def _instances():
    """Every loaded copy of task.py (check.py loads its own), found through the step hooks they registered."""
    seen = {}
    for fn in parts._ON_STEP:
        g = fn.__globals__
        if "observe" in g and "REC" in g:
            seen[id(g["REC"])] = g
    return list(seen.values())


def play(*segments):
    """Reset every copy's trial record, then play (objects, hands, seconds) segments at 20 Hz."""
    t = 0.0
    for g in _instances():
        g["REC"].clear()
        g["REC"].update(g["_fresh"]())
    last = None
    for objects, hands, seconds in segments:
        n = max(1, round(seconds / DT))
        for _ in range(n):
            t += DT
            for g in _instances():
                g["observe"](objects, hands, t)
            last = objects
    return last if last is not None else scene()


def done(*before, final=FULL, hold=1.2):
    return play(*before, (final, FAR, hold))


class _Timelines:
    """check.py iterates CASES in order; each case's timeline is played just before check.py scores it."""

    def __init__(self, cases):
        self.cases = cases

    def __iter__(self):
        for label, build, want, ok in self.cases:
            lot = "B" if label.startswith("lot B") else "A"
            lots.use(lot)
            yield label, build(), want, ok
        lots.use(None)


R = task.BOTTLE_R
XF = task.XF
SPEC = [
    ("no history: nothing observed yet", lambda: play() or scene(), 0.0, False),
    ("arms idle: shelf extended, bottle at the front", lambda: play((scene(), FAR, 2.0)), 0.0, False),
    ("full: tucked, bottle upright in the opened space, hands clear, still 1.2 s", lambda: done(), 1.0, True),
    ("full but still for only 0.85 s", lambda: done(hold=0.85), 0.0, False),
    ("front half pushed back only 130 mm, bottle upright in the space in front of it", lambda: done(final=scene(
        slide=0.130, level=-task.DROP, bottle=bay())), 1.0, True),
    ("bottle still at the front, front half tucked 5.0 mm short of the stop (limit)", lambda: done(final=scene(
        slide=task.TRAVEL - 0.005, level=-task.DROP)), 0.5, False),
    ("bottle still at the front, front half 5.5 mm short of the stop: lowered only", lambda: done(final=scene(
        slide=task.TRAVEL - 0.0055, level=-task.DROP)), 0.25, False),
    ("at the stop but riding 3 mm above the lower track (limit)", lambda: done(final=scene(slide=task.TRAVEL, level=-task.DROP + 0.003,
                                                                                         bottle=bay())), 1.0, True),
    ("at the stop but 3.5 mm above the lower track", lambda: done(final=scene(slide=task.TRAVEL, level=-task.DROP + 0.0035,
                                                                            bottle=bay())), 0.0, False),
    ("lifted, drawn forward and lowered onto the track, not pushed back", lambda: done(final=scene(slide=-task.PULL,
                                                                                                level=-task.DROP)), 0.25, False),
    ("lowered and pushed halfway back", lambda: done(final=scene(slide=0.09, level=-task.DROP)), 0.25, False),
    ("lifted, drawn forward and let go while still lifted on the seats' line", lambda: done(final=scene(slide=-0.010, level=0.005)), 0.0, False),
    ("naive: pushed straight back, stopped by the back half after 2 mm", lambda: done(final=scene(slide=0.002, level=0.0)), 0.0, False),
    ("naive: lifted and pushed back, stopped by the back half's trim", lambda: done(final=scene(slide=0.002, level=task.LIFT)), 0.0, False),
    ("naive: bottle laid on its side under the extended shelf", lambda: done(final=scene(bottle=(XF + 0.1, 0.06, R - 0.0005),
                                                                                        tilt=90.0)), 0.0, False),
    ("tucked, bottle still standing at the front", lambda: done(final=scene(**TUCK)), 0.5, False),
    ("tucked, bottle lying in the opened space", lambda: done(final=scene(**TUCK, bottle=(XF + 0.10, 0.06, R - 0.0005),
                                                                         tilt=90.0)), 0.5, False),
    ("tucked, bottle upright with its base exactly on the front line (limit)",
     lambda: done(final=scene(**TUCK, bottle=(XF + R, 0.06, 0.0005))), 1.0, True),
    ("tucked, bottle base 1 mm over the front line, centre behind it",
     lambda: done(final=scene(**TUCK, bottle=(XF + R - 0.001, 0.06, 0.0005))), 0.75, False),
    ("tucked, bottle centre exactly on the front line (limit)", lambda: done(final=scene(**TUCK, bottle=(XF, 0.06, 0.0005))), 0.75, False),
    ("tucked, bottle centre 1 mm in front of the line", lambda: done(final=scene(**TUCK, bottle=(XF - 0.001, 0.06, 0.0005))), 0.5, False),
    ("tucked, bottle in place leaning 5 degrees (limit)", lambda: done(final=scene(**TUCK, bottle=bay(), tilt=5.0)), 1.0, True),
    ("tucked, bottle in place leaning 5.5 degrees", lambda: done(final=scene(**TUCK, bottle=bay(), tilt=5.5)), 0.5, False),
    ("tucked, bottle base 3 mm above the shelf (limit)", lambda: done(final=scene(**TUCK, bottle=bay(dz=0.003))), 1.0, True),
    ("tucked, bottle held 3.5 mm above the shelf", lambda: done(final=scene(**TUCK, bottle=bay(dz=0.0035))), 0.5, False),
    ("full but a fingertip 3 mm from the bottle", lambda: play((FULL, near(FULL, "bottle"), 2.0)), 0.0, False),
    ("full but a fingertip 3 mm above the front half's trim", lambda: play((FULL, near(FULL, "shelf"), 2.0)), 0.0, False),
    ("1.0 earned, then a fingertip comes back to the bottle", lambda: play((FULL, FAR, 1.5), (FULL, near(FULL, "bottle"), 0.5)), 0.0, False),
    ("hand state unknown at the end", lambda: play((FULL, FAR, 1.5), (FULL, None, 0.1)), 0.0, False),
    ("bottle still sliding at 1 cm/s", lambda: done(final=scene(**TUCK, bottle=bay(), vel=(0.01, 0.0, 0.0))), 0.0, False),
    ("bottle knocked off the shelf to the floor earlier, then a full end state",
     lambda: done((scene(bottle=(0.02, 0.06, -0.060), tilt=90.0), FAR, 0.2)), 0.0, False),
    ("bottle tipped over toward the robot, its middle past the lower shelf's front edge, then a full end state",
     lambda: done((scene(bottle=(0.125, 0.06, R - 0.0005), tilt=-90.0), FAR, 0.2)), 0.0, False),
    ("bottle tipped over toward the robot, its middle 1 mm behind the front edge (limit), then a full end state",
     lambda: done((scene(bottle=(task.SHELF_FRONT + task.BOTTLE_MID + 0.001, 0.06, R - 0.0005), tilt=-90.0), FAR, 0.2)), 1.0, True),
    ("bottle knocked over earlier but stood back up in place", lambda: done((scene(bottle=(XF - 0.075, 0.06, R - 0.0005), tilt=90.0), FAR, 0.5)), 1.0, True),
    ("lot B: full in lot B's layout", lambda: done(final=scene(**TUCK, bottle=bay("B"), lot="B")), 1.0, True),
    ("lot B: tucked, bottle still at lot B's start", lambda: done(final=scene(**TUCK, lot="B")), 0.5, False),
]

CASES = _Timelines(SPEC)


def run():
    bad = 0
    for label, objects, want, ok in CASES:
        got = task.score(objects)
        good = abs(got - want) < 1e-9 and task.success(objects) == ok
        bad += not good
        print("ok  " if good else "FAIL", f"{got:.2f} (want {want:.2f})", label)
    return bad


if __name__ == "__main__":
    raise SystemExit(run())
