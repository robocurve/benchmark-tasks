"""Near-successes and unintended strategies, checked offline.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so before the
cases this file writes into the task's per-trial record what the step callback records for a trial whose end state
has held, let go and still, for 1 s (the grippers 50 mm away). _run() then drives the step-side function observe()
directly to check the 1 s hold, stillness, the let-go distance (with face sampling), the whole-trial rule, the
missing-measurement rule and a tilted carry, and raises if any scores wrongly.
"""

import math

import numpy as np

import lots
import task
from task import CMA, OPEN0, P_INST, RACK
from yam_arena.analysis import Hull

lots.use("A")
SAG = 0.003 / 0.39  # rad: the hinge end sagged 3 mm onto the support bar, pivoting about the clips


def quat(axis, deg):
    a = math.radians(deg) / 2
    v = np.asarray(axis, float) * math.sin(a)
    return (float(v[0]), float(v[1]), float(v[2]), math.cos(a))


def _mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def scene(dx=0.0, dy=0.0, dz=0.0, yaw_deg=0.0, sag=SAG, elbow=0.0, inner=0.0, outer=0.0, gate=0.0):
    """The CMA installed (its frame at the installed pose, moved by dx/dy/dz and turned yaw_deg, its hinge end sagged
    ``sag`` rad about the clips), the outer arm ``elbow`` rad open from folded, the latch teeth and stop bracket
    at the given angles."""
    clip = np.array([0.0, 0.405, 0.0])  # the line the CMA sags about (between the clips), CMA frame
    r = task.parts._rot((1.0, 0.0, 0.0), sag)
    pos = P_INST + clip - r @ clip + (dx, dy, dz)
    q = _mul(quat((0, 0, 1), yaw_deg), quat((1, 0, 0), math.degrees(sag)))
    cma = CMA.state(pos=tuple(float(v) for v in pos), quat_xyzw=q, joints={"elbow": -OPEN0 + elbow})
    rack = RACK.state(joints={"inner_latch": inner, "outer_latch": outer, "stop_bracket": gate})
    return {"rack": rack, "cma": cma}


def preset():
    task.reset_record()
    task._rec.update(steps=50, gap=0.05, since=9.0, level=1.0, now=1.0, t=10.0)


preset()
START = {"rack": RACK.state(), "cma": CMA.state()}
OPEN_GATE = 1.6
CASES = [
    ("installed: both clips latched, resting on the bar, stop bracket shut, let go, 1 s", scene(), 1.0, True),
    ("the same with the stop bracket left open", scene(gate=OPEN_GATE), 2 / 3, False),
    ("stop bracket at 0.15 rad exactly", scene(gate=0.15), 1.0, True),
    ("stop bracket at 0.16 rad", scene(gate=0.16), 2 / 3, False),
    ("both clips latched but the hinge end held 3 mm up off the bar (not resting)", scene(sag=-0.0025, gate=0.0),
     1 / 3, False),
    ("outer arm left 3 degrees open: only the inner clip on", scene(elbow=math.radians(3.0), gate=OPEN_GATE), 1 / 3,
     False),
    ("outer arm left 15 degrees open, stop bracket shut", scene(elbow=OPEN0, gate=0.0), 1 / 3, False),
    ("inner clip 3.5 mm short of home (tooth still in the window)", scene(dx=-0.0035),
     1.0, True),
    ("inner tooth not up (0.3 rad): clip pushed but not snapped", scene(inner=0.30), 1 / 3, False),
    ("outer tooth not up (0.3 rad)", scene(outer=0.30), 1 / 3, False),
    ("both teeth folded (clips pressed but never snapped)", scene(inner=1.2, outer=1.2), 0.0, False),
    ("whole CMA 2.4 mm to the left of the tabs' lines", scene(dy=0.0024), 1.0, True),
    ("whole CMA 3 mm to the left: neither clip on its tab", scene(dy=0.003), 0.0, False),
    ("whole CMA 1.5 mm high at the clips", scene(dz=0.0015, sag=SAG + 0.0015 / 0.39), 1.0, True),
    ("CMA turned 3 degrees about its hinge: clips off line", scene(yaw_deg=3.0), 0.0, False),
    ("inner clip pushed onto the outer tab (wrong tab)", scene(dx=task.X_OUT - task.X_IN, dy=task.Y_OUT - task.Y_IN),
     0.0, False),
    ("CMA on the bench at its start, stop bracket shut (do nothing)", START, 0.0, False),
    ("CMA set on the bench right in front of the rack", {"rack": RACK.state(), "cma": CMA.state(pos=(0.30, -0.205, task.START_A[2]))},
     0.0, False),
]


def _states(**kw):
    out = scene()
    out.update(kw)
    return out


def box_hull(center, half):
    half = np.asarray(half, float)
    v = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half + np.asarray(center)
    planes = []
    for i in range(3):
        for sign in (-1.0, 1.0):
            n = np.zeros(3)
            n[i] = sign
            planes.append([*n, sign * center[i] + half[i]])
    return Hull(vertices=v, planes=np.array(planes))


def _check(label, got, want):
    if abs(got - want) > 1e-9:
        raise AssertionError(f"{label}: score {got}, expected {want}")


def _run() -> None:
    dt = 0.05
    end = scene()
    far = 0.05

    task.reset_record()
    for k in range(20):
        task.observe(end, far, k * dt)
    _check("0.95 s into the hold", task.score(end), 0.0)
    task.observe(end, far, 20 * dt)
    _check("1.0 s into the hold", task.score(end), 1.0)
    task.observe(end, 0.029, 21 * dt)
    _check("a finger 29 mm from the CMA", task.score(end), 0.0)
    for k in range(22, 50):
        task.observe(end, 0.0299, k * dt)
    _check("a finger 29.9 mm away: not let go", task.score(end), 0.0)
    for k in range(50, 80):
        task.observe(end, 0.0301, k * dt)
    _check("every finger just over 30 mm away", task.score(end), 1.0)

    # Held: never credited, even when installed.
    task.reset_record()
    for k in range(40):
        task.observe(end, 0.0, k * dt)
    _check("installed but still held", task.score(end), 0.0)

    # Stillness: the CMA rocking 3 mm to and fro within the second does not count.
    task.reset_record()
    for k in range(30):
        s = scene(dx=-0.003 * (k % 2))
        task.observe(s, far, k * dt)
    _check("CMA rocking 3 mm", task.score(s), 0.0)

    # The tier must hold for the whole second: full for 0.5 s, then the stop bracket swings open.
    task.reset_record()
    for k in range(10):
        task.observe(end, far, k * dt)
    opened = scene(gate=1.6)
    for k in range(10, 40):
        task.observe(opened, far, k * dt)
    _check("stop bracket swung open after", task.score(opened), 2 / 3)

    # Whole-trial rules.
    task.reset_record()
    task.observe(_states(cma=CMA.state(pos=(0.25, -0.205, task.TOP - 0.05))), far, 0.0)
    for k in range(1, 40):
        task.observe(end, far, k * dt)
    _check("CMA dropped off the bench once", task.score(end), 0.0)
    task.reset_record()
    task.observe(_states(cma=CMA.state(pos=(0.25, -0.205, task.TOP + 0.15), quat_xyzw=quat((1, 0, 0), 50.0))), 0.0,
                 0.0)
    for k in range(1, 40):
        task.observe(end, far, k * dt)
    _check("CMA held tilted 50 degrees on the way, then installed", task.score(end), 1.0)
    task.reset_record()
    task.observe(end, None, 0.0)
    for k in range(1, 40):
        task.observe(end, far, k * dt)
    _check("grippers unreadable at one step", task.score(end), 0.0)
    task.reset_record()
    _check("no step recorded", task.score(end), 0.0)

    # Let-go is measured on sampled hull faces: a 180 x 120 mm face 20 mm over the CMA, every corner
    # more than 30 mm away.
    c = end["cma"]
    top = float(c.high[2])
    h = box_hull((0.390, 0.0, top + 0.020 + 0.01), (0.090, 0.060, 0.010))
    corners_only = task.gap_to(end, h.vertices)
    sampled = task.gap_to(end, task._face_points(h))
    if not (corners_only > task.CLEAR and sampled < task.CLEAR):
        raise AssertionError(f"face sampling: corners {corners_only:.4f} m, sampled {sampled:.4f} m")

    preset()


_run()
