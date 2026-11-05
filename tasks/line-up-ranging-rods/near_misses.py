"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None)}, expected score, expected success).
"""

import numpy as np

from task import LINE_LEN, POLE_A, POLE_B, ROD_H, ROD_R, TOP

_A, _B = np.array(POLE_A), np.array(POLE_B)
_U = (_B - _A) / LINE_LEN
_N = np.array([-_U[1], _U[0]])
Z = TOP + ROD_H / 2


def at(t, off=0.0, z=Z):
    """A rod standing t metres along the line from pole A's centre, off metres to one side of it."""
    p = _A + t * _U + off * _N
    return (float(p[0]), float(p[1]), z)


def three(t1, t2, t3, o1=0.0, o2=0.0, o3=0.0):
    return {"rod_1": (at(t1, o1), None, None), "rod_2": (at(t2, o2), None, None), "rod_3": (at(t3, o3), None, None)}


Q = LINE_LEN / 4  # 111.8 mm: the quarter points
EVEN = (Q, 2 * Q, 3 * Q)
LYING = (ROD_H, 2 * ROD_R, 2 * ROD_R)
MOVING = (0.0, 0.0, -0.05)


def lying_at(t):
    return (at(t, z=TOP + ROD_R), LYING, None)


CASES = [
    ("quarter points, on the line", three(*EVEN), 1.0, True),
    ("one rod 5 mm off", three(*EVEN, o2=0.005), 1.0, True),
    ("last rod still held 2 mm above the table", {**three(*EVEN), "rod_3": (at(3 * Q, z=Z + 0.002), None, None)},
     0.0, False),
    ("last rod settled 0.8 mm proud (contact noise)", {**three(*EVEN), "rod_3": (at(3 * Q, z=Z + 0.0008), None, None)},
     1.0, True),
    ("one rod 6 mm off", three(*EVEN, o2=-0.006), 0.6, False),
    ("worst rod 10 mm off", three(*EVEN, 0.004, -0.010, 0.003), 0.6, False),
    ("worst rod 12 mm off", three(*EVEN, o3=0.012), 0.3, False),
    ("worst rod 20 mm off", three(*EVEN, o1=-0.020), 0.3, False),
    ("one rod 21 mm off", three(*EVEN, o1=0.021), 0.0, False),
    ("uneven but every gap 70 to 160 mm", three(0.100, 0.200, 0.340), 1.0, True),
    ("first gap exactly 70 mm", three(0.070, 0.200, 0.340), 1.0, True),
    ("first gap 65 mm (crowding pole A)", three(0.065, 0.200, 0.340), 0.0, False),
    ("two rods 60 mm apart", three(0.150, 0.210, 0.340), 0.0, False),
    ("rods bunched in the middle: end gaps 170 mm", three(0.170, 0.2235, 0.277), 0.0, False),
    ("all three touching in a cluster", three(0.200, 0.224, 0.248), 0.0, False),
    ("one rod past pole B, on the extended line", three(Q, 2 * Q, LINE_LEN + 0.08), 0.0, False),
    ("one rod before pole A, on the extended line", three(-0.08, 2 * Q, 3 * Q), 0.0, False),
    ("one rod lying on the line", {**three(*EVEN), "rod_2": lying_at(2 * Q)}, 0.0, False),
    ("one rod still moving", {**three(*EVEN), "rod_3": (at(3 * Q), None, MOVING)}, 0.0, False),
    ("one rod held 10 mm above the table", {**three(*EVEN), "rod_1": (at(Q, z=Z + 0.010), None, None)}, 0.0, False),
    ("only two rods placed, third at its start", {k: v for k, v in three(*EVEN).items() if k != "rod_3"}, 0.0, False),
    ("one rod stacked on another", {**three(*EVEN), "rod_3": (at(2 * Q, z=Z + ROD_H), None, None)}, 0.0, False),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(); print and count mismatches."""
    import task
    from yam_arena.task_file import ObjectState

    def state(pos, size, vel=None):
        pos, half = np.asarray(pos, float), np.asarray(size, float) / 2
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), v, np.zeros(3), pos - half, pos + half)

    sizes = {o.name: (2 * o.radius, 2 * o.radius, o.height) for o in task.OBJECTS}
    start = {o.name: state(o.pos, sizes[o.name]) for o in task.OBJECTS}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for name, (pos, size, vel) in changes.items():
            st[name] = state(pos, sizes[name] if size is None else size, vel)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
