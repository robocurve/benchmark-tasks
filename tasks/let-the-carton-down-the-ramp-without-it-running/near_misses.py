"""Near-successes and unintended strategies, each with the score task.py must give it. Cases run in order against one
copy of the task module, so the whole-trial record (a moved witness carton, a carton that touched the table) carries
from case to case until a scene reset puts every loose object back at its start."""

import math

import numpy as np

from task import (CARTON, CARTON_SIZE, DOCK, DOCK_T, DOCK_W, SLOPE, TOP, W, WITNESSES, XC, Y_DOCK_END, Y_FOOT, Y_EDGE,
                  BED_TOP)

H = CARTON_SIZE[2] / 2
WA, WB = WITNESSES


def quat_x(angle):
    return np.array([math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2)])


def carton(y, x=XC, lift=0.0, tilt=0.0, vel=None):
    """Carton upright on the dock (unless tilted about x by tilt rad), its centre at (x, y)."""
    q = quat_x(tilt)
    hz = H * math.cos(tilt) + CARTON_SIZE[1] / 2 * abs(math.sin(tilt))  # half height of its box
    z = TOP + DOCK_T + hz + lift
    return CARTON.state(pos=(x, y, z), quat_xyzw=q, lin_vel=vel)


def on_slide(u=0.10):
    """Carton resting on the slide, its centre u m down the slope from the edge."""
    q = quat_x(SLOPE)  # turned about +x so its downhill (-y) end is low
    y = Y_EDGE - u * math.cos(SLOPE) - H * math.sin(SLOPE)
    z = TOP + BED_TOP - u * math.sin(SLOPE) + H * math.cos(SLOPE)
    return CARTON.state(pos=(x := XC, y, z), quat_xyzw=q)


def on_table(y=-0.32, x=XC):
    return CARTON.state(pos=(x, y, TOP + H))


def witness(spec, dx=0.0, dy=0.0):
    p = np.asarray(spec.pos, float) + np.array([dx, dy, 0.0])
    return (tuple(p), spec.size, None)


Y_IN = Y_FOOT - CARTON_SIZE[1] / 2 - 0.01  # well on the dock
Y_UP_LIMIT = Y_FOOT - CARTON_SIZE[1] / 2  # uphill face exactly at the foot
Y_DOWN_LIMIT = Y_DOCK_END + CARTON_SIZE[1] / 2  # downhill face exactly at the dock's end
X_LIMIT = XC + DOCK_W / 2 - CARTON_SIZE[0] / 2

DONE = {"carton": carton(Y_IN)}

SINGLE = [
    ("carton upright on the dock, still, witnesses untouched", DONE, 1.0, True),
    ("carton still on the trailer bed (do nothing)", {}, 0.0, False),
    ("carton resting partway down the slide", {"carton": on_slide(0.10)}, 0.0, False),
    ("uphill face 1 mm onto the dock past the foot", {"carton": carton(Y_UP_LIMIT - 0.001)}, 1.0, True),
    ("uphill edge resting on the slide, tilted 5 degrees (straddling the foot)",
     {"carton": carton(Y_UP_LIMIT + 0.01, tilt=math.radians(5.0))}, 0.0, False),
    ("straddling the foot at the slide's 30 degrees", {"carton": carton(Y_UP_LIMIT + 0.02, tilt=SLOPE)}, 0.0, False),
    ("downhill face exactly at the dock's end", {"carton": carton(Y_DOWN_LIMIT)}, 1.0, True),
    ("downhill face 1 mm past the dock's end", {"carton": carton(Y_DOWN_LIMIT - 0.001)}, 0.0, False),
    ("side exactly at the dock's side edge", {"carton": carton(Y_IN, x=X_LIMIT)}, 1.0, True),
    ("side 1 mm past the dock's side edge", {"carton": carton(Y_IN, x=X_LIMIT + 0.001)}, 0.0, False),
    ("held 2 mm above the dock", {"carton": carton(Y_IN, lift=0.002)}, 0.0, False),
    ("1 mm above the dock (within tolerance)", {"carton": carton(Y_IN, lift=0.001)}, 1.0, True),
    ("still sliding across the dock", {"carton": carton(Y_IN, vel=(0.0, -0.05, 0.0))}, 0.0, False),
    ("tilted 9.9 degrees", {"carton": carton(Y_IN - 0.005, tilt=math.radians(9.9))}, 1.0, True),
    ("tilted 10.1 degrees", {"carton": carton(Y_IN - 0.005, tilt=math.radians(10.1))}, 0.0, False),
    ("tipped onto its side on the dock", {"carton": CARTON.state(pos=(XC, Y_IN, TOP + DOCK_T + CARTON_SIZE[1] / 2),
                                                                 quat_xyzw=quat_x(math.pi / 2))}, 0.0, False),
    ("on the dock, but witness A was pushed 3.1 mm", {**DONE, "witness_a": witness(WA, dy=-0.0031)}, 0.0, False),
    ("on the dock, witness B moved exactly 3 mm", {**DONE, "witness_b": witness(WB, dx=0.003)}, 1.0, True),
    ("witnesses pushed aside out of the way first", {**DONE, "witness_a": witness(WA, dx=-0.10),
                                                     "witness_b": witness(WB, dx=0.10)}, 0.0, False),
]

CHAIN = [
    ("carton let go on the slide, runs into the witness cartons", {"carton": carton(-0.20),
                                                                   "witness_a": witness(WA, dy=-0.04)}, 0.0, False),
    ("then witness put back and carton on the dock: the knock still counts", DONE, 0.0, False),
    ("scene reset: everything back at its start", {}, 0.0, False),
    ("after the reset, done again", DONE, 1.0, True),
    ("scene reset again", {}, 0.0, False),
    ("carton knocked off the slide's side onto the table", {"carton": on_table(0.0, XC + 0.15)}, 0.0, False),
    ("then lifted onto the dock: the fall still counts", DONE, 0.0, False),
    ("scene reset again", {}, 0.0, False),
    ("carton runs off the dock's end onto the table, witnesses untouched", {"carton": on_table(-0.30, XC + 0.15)},
     0.0, False),
    ("then pushed back onto the dock: the fall still counts", DONE, 0.0, False),
]



def stepped(d):
    """The carton on the dock, its centre d m further along than in DONE (one control step later it is at DONE)."""
    return {"carton": carton(Y_IN + d)}


CHAIN += [
    ("scene reset again", {}, 0.0, False),
    ("carton on the dock", stepped(0.0201), 0.0, False),
    ("next step it is 20.1 mm further on: it ran, even though it then rests on the dock", DONE, 0.0, False),
    ("scene reset again", {}, 0.0, False),
    ("carton on the dock", stepped(0.0199), 0.0, False),
    ("next step it is 19.9 mm further on: counts", DONE, 1.0, True),
]

RESET = ("(scene reset: everything back at its start)", {}, 0.0, False)
CASES = [c for case in SINGLE for c in (RESET, case)] + [RESET] + CHAIN


def _run() -> int:
    """Score every case in order with task.py's own score() and success(); print and count mismatches."""
    import task
    from yam_arena.task_file import ObjectState

    def box_state(pos, size, vel=None):
        p = np.asarray(pos, float)
        h = np.asarray(size, float) / 2
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(p, np.array([0.0, 0.0, 0.0, 1.0]), v, np.zeros(3), p - h, p + h)

    start = {"dock": DOCK.state(joints={"slide": SLOPE}), "carton": CARTON.state()}
    for w in WITNESSES:
        start[w.name] = box_state(w.pos, w.size)
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for k, v in changes.items():
            st[k] = box_state(*v) if isinstance(v, tuple) else v
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
