"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so before the
cases this file writes into the task's per-step record what the step callback would have measured for a still end
state with both grippers 50 mm away, held for 2 s. _run() below then drives the step-side functions directly (the
catch, the friction drive, the whole-trial rules, the release distance from gripper faces, the stillness test, the
2 s hold and the missing-measurement rule) in both lots, and raises if any scores wrongly.
"""

import math

import numpy as np

import lots
import task
from task import COOLER, OPEN_Q, PRESSED

DEG = math.radians


def quat(axis, deg):
    a = math.radians(deg) / 2
    v = np.asarray(axis, float) * math.sin(a)
    return (float(v[0]), float(v[1]), float(v[2]), math.cos(a))


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def start_of():
    return tuple(float(v) for v in lots.current().pos("cooler"))


def cooler(q=OPEN_Q, button=0.0, shift=(0.0, 0.0, 0.0), tilt_deg=0.0, vel=None):
    lot = lots.current()
    p = np.asarray(lot.pos("cooler"), float) + np.asarray(shift, float)
    qz = quat((0, 0, 1), math.degrees(lot.yaw_of("cooler")))
    qx = qmul(qz, quat((1, 0, 0), tilt_deg)) if tilt_deg else qz
    return {"cooler": COOLER.state(joints={"lid": q, "button": button}, pos=tuple(p), quat_xyzw=qx, lin_vel=vel)}


def away():
    """What the step callback records for a still, released end state held 2 s: grippers 50 mm away."""
    task._rec.update(fault=False, gap=0.05, t=10.0, since=8.0, held=True, prev=None, start=start_of())


lots.use("A")
away()
CASES = [
    ("lid open on its stop, button out, let go and still", cooler(), 1.0, True),
    ("lid open the other way, on its other stop", cooler(-OPEN_Q), 1.0, True),
    ("lid exactly 80 degrees open", cooler(DEG(80.0)), 1.0, True),
    ("lid 79 degrees open", cooler(DEG(79.0)), 0.0, False),
    ("lid partly open, 30 degrees", cooler(DEG(30.0)), 0.0, False),
    ("lid shut (do nothing)", cooler(0.0), 0.0, False),
    ("lid shut, button held in (button alone)", cooler(0.0, PRESSED), 0.0, False),
    ("lid open, button still held in", cooler(button=0.002), 0.0, False),
    ("lid open, button in exactly 1 mm", cooler(button=0.001), 1.0, True),
    ("lid open, cooler slid exactly 10 mm", cooler(shift=(0.010, 0.0, 0.0)), 1.0, True),
    ("lid open, cooler slid 11 mm", cooler(shift=(0.0, 0.011, 0.0)), 0.0, False),
    ("lid open, cooler tilted 3 degrees", cooler(tilt_deg=3.0), 1.0, True),
    ("lid open, cooler tilted 4 degrees (propped)", cooler(tilt_deg=4.0), 0.0, False),
    ("lid open, cooler on its side", cooler(tilt_deg=90.0), 0.0, False),
    ("lid open, cooler still sliding", cooler(vel=(0.0, 0.02, 0.0)), 0.0, False),
]


def box_hull(center, half, rot=np.eye(3)):
    """A gripper part as a box hull (world frame)."""
    half = np.asarray(half, float)
    center = np.asarray(center, float)
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    verts = corners @ rot.T + center
    planes = []
    for i in range(3):
        for sign in (-1.0, 1.0):
            n = sign * rot[:, i]
            planes.append([*n, float(n @ (center + sign * half[i] * rot[:, i]))])

    class _H:
        pass

    h = _H()
    h.vertices, h.planes = verts, np.array(planes)
    return h


def _catch_cases(bad):
    ns = task.next_state
    want = [
        (("locked", 0.0, 0.0), "locked", "shut, button out"),
        (("locked", 0.0, PRESSED), "free", "shut, button in 5 mm"),
        (("locked", 0.0, PRESSED - 0.0001), "locked", "shut, button in 4.9 mm"),
        (("free", DEG(5.0), 0.0), "free", "lid 5 degrees off, button let go"),
        (("free", DEG(0.5), 0.0), "locked", "lid back within 0.5 degrees, button out (clicks shut)"),
        (("free", DEG(-0.6), 0.0), "free", "lid 0.6 degrees off the other way, button out"),
        (("free", DEG(90.0), PRESSED), "free", "open with the button in"),
    ]
    for args, w, label in want:
        if ns(*args) != w:
            bad.append(f"catch: {label}: {ns(*args)} (want {w})")
    for a0, q, want_a in ((0.0, 0.01, 0.0), (0.0, 0.03, 0.01), (0.0, -0.05, -0.03), (0.2, 0.19, 0.2)):
        if abs(task.drag_anchor(a0, q) - want_a) > 1e-9:
            bad.append(f"drag: anchor {a0}, lid {q}: {task.drag_anchor(a0, q)} (want {want_a})")


def _fault_cases(bad):
    for lot in ("A", "B"):
        lots.use(lot)
        s = start_of()
        for st, want, label in (
            (cooler(0.0, tilt_deg=10.0)["cooler"], False, "tilted exactly 10 degrees"),
            (cooler(0.0, tilt_deg=10.5)["cooler"], True, "tilted 10.5 degrees"),
            (cooler(0.0, shift=(0.0, 0.0, 0.010))["cooler"], False, "lifted exactly 10 mm"),
            (cooler(0.0, shift=(0.0, 0.0, 0.011))["cooler"], True, "lifted 11 mm"),
            (cooler(0.0, shift=(0.05, 0.0, 0.0))["cooler"], False, "slid 50 mm on the mat (judged at the end)"),
        ):
            if task.fault(st, s) != want:
                bad.append(f"lot {lot}: whole-trial rule: {label}: {task.fault(st, s)} (want {want})")
    lots.use("A")


def _still_cases(bad):
    k = cooler()["cooler"]
    q = float(k.joints["lid"])
    for prev, dt, want, label in (
        ((q, 0.0), 0.05, True, "nothing moving"),
        ((q - 0.002, 0.0), 0.05, True, "lid turning 0.04 rad/s"),
        ((q - 0.003, 0.0), 0.05, False, "lid turning 0.06 rad/s"),
        ((q, 0.0003), 0.05, False, "button moving 6 mm/s"),
        (None, 0.05, False, "no previous step"),
    ):
        if task.still(k, prev, dt) != want:
            bad.append(f"stillness: {label}: {task.still(k, prev, dt)} (want {want})")


def _run():
    bad = []
    lots.use("A")
    done = cooler()["cooler"]

    def fresh():
        task._rec.update(fault=False, gap=None, t=None, since=None, held=False, prev=None, start=start_of())

    # The hold: 1.95 s is not enough, 2.0 s is.
    for hold_t, want in ((1.95, 0.0), (2.0, 1.0)):
        fresh()
        for t in (0.0, hold_t):
            task._rec.update(gap=0.05)
            task._hold(done, t, True)
        if task.score({"cooler": done}) != want:
            bad.append(f"held {hold_t} s: {task.score({'cooler': done})} (want {want})")
    # A hold broken part way restarts.
    fresh()
    for t, ok in ((0.0, True), (1.5, False), (2.0, True), (3.9, True)):
        task._rec.update(gap=0.05)
        task._hold(done, t, ok)
    if task.score({"cooler": done}) != 0.0:
        bad.append("hold broken at 1.5 s, then 1.9 s more: scored")
    # Release distance: 6 mm counts; 4 mm and exactly 5 mm do not.
    for g, want in ((0.006, 1.0), (0.004, 0.0), (task.CLEAR, 0.0)):
        fresh()
        task._rec.update(gap=g, since=0.0, held=True)
        if task.score({"cooler": done}) != want:
            bad.append(f"gap {g * 1000:.1f} mm: {task.score({'cooler': done})} (want {want})")
    # No gripper measurement, or no start record: no credit.
    fresh()
    task._rec.update(since=0.0, held=True)
    if task.score({"cooler": done}) != 0.0:
        bad.append("no gripper measurement: scored")
    fresh()
    task._rec.update(gap=0.05, since=0.0, held=True, start=None)
    if task.score({"cooler": done}) != 0.0:
        bad.append("no start record: scored")
    # Tipped earlier in the trial (whole-trial rule), then stood back up and opened: 0.
    fresh()
    task._rec.update(gap=0.05, since=0.0, held=True, fault=True)
    if task.score({"cooler": done}) != 0.0:
        bad.append("tipped earlier, then a good finish: scored")
    # A non-finite state: no credit.
    bad_state = COOLER.state(joints={"lid": float("nan"), "button": 0.0}, pos=done.pos, quat_xyzw=done.quat_xyzw)
    fresh()
    task._rec.update(gap=0.05, since=0.0, held=True)
    if task.score({"cooler": bad_state}) != 0.0:
        bad.append("non-finite lid angle: scored")
    # Release measured from gripper faces: a 60 mm plate set square across the body's front-right vertical corner (below
    # the lid's end plates), its face 4 mm from the corner and all its own corners more than 5 mm away, is not let go;
    # 6 mm off it is.
    for lot in ("A", "B"):
        lots.use(lot)
        k = cooler()["cooler"]
        r = task._rot(k.quat_xyzw)
        n_l, t_l = np.array([-1.0, -1.0, 0.0]) / math.sqrt(2), np.array([1.0, -1.0, 0.0]) / math.sqrt(2)
        rot = r @ np.c_[n_l, t_l, [0.0, 0.0, 1.0]]
        edge = np.array([-task.HALF_W, -task.HALF_L, 0.030])
        for dz, want in ((0.004, False), (0.006, True)):
            center = np.asarray(k.pos) + r @ (edge + n_l * (dz + 0.002))
            plate = box_hull(center, (0.002, 0.030, 0.020), rot)
            wh = task._WorldHull(plate, task._face_points(plate), np.eye(4))
            g_faces, g_corners = task.gap(k, [wh]), task.gap(k, [plate])
            if (g_faces > task.CLEAR) != want or g_corners <= task.CLEAR:
                bad.append(f"lot {lot}: face {dz * 1000:.0f} mm off the body's corner: gap {g_faces * 1000:.1f} mm "
                           f"from faces, {g_corners * 1000:.1f} mm from corners")
    lots.use("A")
    _catch_cases(bad)
    _fault_cases(bad)
    _still_cases(bad)
    away()
    if bad:
        raise AssertionError("; ".join(bad))


_run()

if __name__ == "__main__":
    for label, states, want, _ in CASES:
        away()
        print(f"{task.score(states):.2f} (want {want:.2f})  {label}")
    print("rule checks passed")
