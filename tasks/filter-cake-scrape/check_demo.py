"""Offline checks, no Isaac Sim or GPU: the demo's IK, the gripper opening, and the scorer on made-up states.

    python tasks/filter-cake-scrape/check_demo.py
"""

import math
from pathlib import Path

import numpy as np

from yam_arena import station, task_file
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import ArmPlan

TASK = Path(__file__).with_name("filter_cake_scrape.py")
DT = 1.0 / 20
task = task_file.load(TASK)
m = task.module
ok = True

# 1) the demo is within reach
for arm, waypoints in task.demo().items():
    plan = ArmPlan(list(waypoints), station.BASES[arm])
    for t in np.arange(0.0, plan.duration + DT, DT):
        plan.action7(float(t))
    good = plan.worst_ik_mm < 5.0 and plan.worst_ik_deg < 3.0
    ok &= good
    print(f"IK {arm:5s}: {plan.duration:5.1f} s, worst {plan.worst_ik_mm:.1f} mm / {plan.worst_ik_deg:.1f} deg  {'PASS' if good else 'FAIL'}")

# 2) what the gripper must close across
print(f"gripper: FINGER_OPEN={FINGER_OPEN} FINGER_CLOSED={FINGER_CLOSED} PAD_TIP_PAST_TCP={PAD_TIP_PAST_TCP}")
print(f"  closes across {m.TRAY_W * 100:.1f} cm (tray) and {m.SPATULA.size[1] * 100:.1f} cm (spatula); the 4 cm wash bottle grasped fine")


# 3) the scorer
def st(spec, pos=None, yaw_deg=0.0):
    pos = np.array(spec.pos if pos is None else pos, dtype=float)
    h = np.array(spec.size) / 2
    half = math.radians(yaw_deg) / 2
    return task_file.ObjectState(pos, np.array([0, 0, math.sin(half), math.cos(half)]), np.zeros(3), np.zeros(3), pos - h, pos + h)


def objs(n_in=0, tray=None):
    out = {o.name: st(o) for o in m.FREE.values()}
    for i, c in enumerate(m.CAKE[:n_in]):
        dx, dy = (i % 3 - 1) * 0.014, (i // 3 - 1.5) * 0.013
        out[c.name] = st(c, (m.BIN_X + dx, (m.BIN_Y_HI + m.BIN_Y_LO) / 2 + dy, m.TOP + m.CUBE / 2))
    if tray is not None:
        out["filter_tray"] = tray
    return out


def shifted(dy):
    p = np.array(m.TRAY.pos) + np.array([0, dy, 0])
    return st(m.TRAY, p)


def run(states):
    m._reset_state()
    for s in states:
        out = m.score(s)
    return out, m.success(states[-1])


cases = {
    "nothing done": ([objs()], 0.0, False),
    "all twelve in, tray steady": ([objs(), objs(12)], 1.0, True),
    "six in, tray steady": ([objs(), objs(6)], 0.5, False),
    "all in, tray slid 2 cm on the way": ([objs()] + [objs(0, shifted(-0.004 * k)) for k in range(6)] + [objs(12, shifted(-0.02))], 0.5, False),
    "all in, tray slid 2 cm and pushed back": (
        [objs()] + [objs(0, shifted(-0.004 * k)) for k in range(6)] + [objs(0, shifted(-0.004 * k)) for k in range(5, -1, -1)] + [objs(12)],
        0.5,
        False,
    ),
    "all in, tray turned 10 degrees": ([objs(), objs(0, st(m.TRAY, None, 10)), objs(12)], 0.5, False),
    "all in, tray shifted 5 mm": ([objs(), objs(12, shifted(-0.005))], 1.0, True),
}
for name, (states, want, want_ok) in cases.items():
    got, got_ok = run(states)
    good = abs(got - want) < 1e-6 and got_ok == want_ok
    ok &= good
    print(f"score {name:44s} {got:.2f} success {got_ok!s:5s} (want {want:.2f} {want_ok!s:5s})  {'PASS' if good else 'FAIL'}")
m._reset_state()
print("all checks passed" if ok else "SOME CHECKS FAILED")
