"""Near-successes and unintended strategies, scored offline by the factory checker.

CASES: (label, {object: ready state}, expected score, expected success). States come from Part.state(pos=...,
quat_xyzw=...): the stopper's origin is the centre of its head's underside, the rod's origin the pivot ball's centre.
Offline there is no simulator, so the movement test cannot run: in CASES the parts' poses stand in for its outcome
(the rod's centreline through the eye's hole or not). READINGS then checks the test's own limits from its readings
(stopper drop when the rod's end is lifted, rise back when it is let go, rise under the knob pull, in mm), or None for
a test not yet run. THROUGH holds the readings measured on the GPU with the rod through the eye. BESIDE and UNDER
follow from where the rod is: in GPU traces of those two outcomes the test never started (README). Offline there is no robot, so the release check passes;
_release_cases() checks the release rule with a gripper near a part and with no measurement, and _test_cases() steps
the test's state machine (the 1 s of stillness before it, abandoned or withdrawn when a gripper comes near).
"""

import math
import os
from dataclasses import replace

import numpy as np

import task
from task import BX, BY, EYE_Z, HF, HP, RD, ROD, SEATED, STOPPER, U, YAW_D

os.environ[task.OFFLINE_FLAG] = "1"  # the task credits these offline cases as let go


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def qz(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def qx(a):
    return (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))


def stopper(yaw=math.pi, eye_z=HP, dx=0.0, dy=0.0, tilt=0.0, vel=None):
    """The stopper turned `yaw` from the drain's frame (pi or 0: its eye facing the port; pi/2: as it starts), with
    its eye's centre at height `eye_z`; dx is sideways across the port, tilt about that sideways axis."""
    off = RD @ np.array([dx, dy, 0.0])
    q = q_mul(q_mul(qz(YAW_D), qx(tilt)), qz(yaw))
    return {"stopper": STOPPER.state(pos=(BX + off[0], BY + off[1], eye_z - EYE_Z), quat_xyzw=q, lin_vel=vel)}


def rod(out=0.0, tilt_deg=0.0, dx=0.0, dz=0.0, vel=None):
    """The rod along the port axis, its ball `out` metres short of seated; tilt_deg lowers the inner tip."""
    p = SEATED + U * out + RD @ np.array([dx, 0.0, dz])
    q = q_mul(qz(YAW_D), qx(-math.radians(tilt_deg)))
    return {"pivot_rod": ROD.state(pos=tuple(float(v) for v in p), quat_xyzw=q, lin_vel=vel)}


def spun(case, w):
    """The rod of a case turning at angular velocity w (world, rad/s)."""
    return {**case, "pivot_rod": replace(case["pivot_rod"], ang_vel=np.asarray(w, float))}


DROP_MM, BACK_MM, RISE_MM = 11.2, 12.0, 6.5  # measured with the rod through the eye in the final GPU demo


def tip_down_eye(tilt_deg):
    """Eye height where a rod tilted tilt_deg (tip down; negative: tip up) meets the eye's top edge."""
    t = math.radians(tilt_deg)
    axis_z = HP - float(np.linalg.norm(SEATED[:2] - np.array([BX, BY]))) * math.tan(t)
    return axis_z + task.ROD_R - task.HOLE


THROUGH = (DROP_MM, BACK_MM, RISE_MM)  # rod through the eye: measured in the final GPU demo
BESIDE = (0.0, 0.0, 99.0)  # rod beside or short of the eye: the stopper sits closed, nothing moves, the pull lifts it
UNDER = (DROP_MM, BACK_MM, 99.0)  # stopper resting on a rod under its eye: it follows the rod, but lifts off
SEATED_CASE = {**stopper(), **rod()}

# Pose cases: offline the parts' poses stand in for the movement test's outcome (rod through the eye's hole or not).
CASES = [
    ("rod through the eye, horizontal, ball home", SEATED_CASE, 1.0, True),
    ("stopper turned the other way (hole axis flipped)", {**stopper(yaw=0.0), **rod()}, 1.0, True),
    ("rest pose: rod tilted 13 deg, eye resting on it", {**stopper(eye_z=tip_down_eye(13)), **rod(tilt_deg=13)},
     1.0, True),
    ("let go: rod's outer end dropped 14 deg, tip holding the stopper up",
     {**stopper(eye_z=tip_down_eye(-14)), **rod(tilt_deg=-14, out=0.0015)}, 1.0, True),
    ("outer end 197 mm from the spout", {**stopper(), **rod(out=0.001)}, 1.0, True),
    ("outer end 199 mm from the spout", {**stopper(), **rod(out=0.003)}, 1.0, True),
    ("outer end 199.5 mm from the spout: through the eye only", {**stopper(), **rod(out=0.0035)}, 0.5, False),
    ("ball 9 mm out, rod still through the eye", {**stopper(), **rod(out=0.009)}, 0.5, False),
    ("rod beside the eye, ball home (stopper not turned, closed)",
     {**stopper(yaw=math.pi / 2, eye_z=HF + EYE_Z), **rod()}, 0.0, False),
    ("rod beside the eye, stopper lifted but not turned", {**stopper(yaw=math.pi / 2), **rod()}, 0.0, False),
    ("rod beside the ring, 11 mm off", {**stopper(dx=0.011), **rod()}, 0.0, False),
    ("rod under the eye, ball home, stopper resting on it",
     {**stopper(eye_z=HP + task.ROD_R + task.HOLE + task.TAB_BAR), **rod()}, 0.0, False),
    ("rod over the eye's ring, ball home, stopper closed", {**stopper(eye_z=HF + EYE_Z), **rod()}, 0.0, False),
    ("rod short of the eye, ball 25 mm out", {**stopper(), **rod(out=0.025)}, 0.0, False),
    ("stopper turned 60 deg from the port", {**stopper(yaw=math.pi + math.radians(60)), **rod()}, 0.0, False),
    ("rod at its start, stopper lifted and turned", {**stopper()}, 0.0, False),
    ("rod crossing the spout's end face 6 mm off its axis", {**stopper(), **rod(dx=0.006)}, 0.0, False),
    ("rod rolling 1.2 rad/s about its own axis: still in place", spun(SEATED_CASE, -1.2 * U), 1.0, True),
    ("rod swinging 0.6 rad/s about the vertical", spun(SEATED_CASE, (0.0, 0.0, 0.6)), 0.0, False),
    ("stopper still moving", {**stopper(vel=(0.0, 0.0, 0.05)), **rod()}, 0.0, False),
    ("stopper pulled 41 mm above the flange", {**stopper(eye_z=HF + 0.041 + EYE_Z), **rod()}, 0.0, False),
    ("stopper tilted 6.5 deg in the bore", {**stopper(tilt=math.radians(6.5)), **rod()}, 0.0, False),
    ("stopper tilted 5.5 deg in the bore", {**stopper(tilt=math.radians(5.5)), **rod()}, 1.0, True),
    ("stopper head 3.9 mm off centre, toward the port", {**stopper(dy=-0.0039), **rod()}, 1.0, True),
    ("stopper head 4.1 mm off centre", {**stopper(dx=0.0041), **rod()}, 0.0, False),
]

# Movement-test cases, all with the parts posed as SEATED_CASE: the test's readings (mm) decide.
READINGS = [
    ("rod through the eye: the GPU readings", THROUGH, 1.0),
    ("rod beside or short of the eye: nothing moves, the pull lifts it", BESIDE, 0.0),
    ("stopper resting on a rod under its eye: follows it, lifts off", UNDER, 0.0),
    ("drop 2.9 mm when the end is lifted", (2.9, BACK_MM, RISE_MM), 0.0),
    ("drop 3.0 mm when the end is lifted", (3.0, BACK_MM, RISE_MM), 1.0),
    ("rises back only 2.9 mm when let go", (DROP_MM, 2.9, RISE_MM), 0.0),
    ("pull rise 10.0 mm", (DROP_MM, BACK_MM, 10.0), 1.0),
    ("pull rise 10.1 mm", (DROP_MM, BACK_MM, 10.1), 0.0),
    ("test not yet run", None, 0.0),
]


def _set_test(readings, home=True):
    task._TEST.clear()
    if readings is None:
        task._TEST.update(phase="wait", k=0)
    else:
        task._TEST.update(phase="done", k=0, seated=home, passed=task.test_passes(*(v / 1000.0 for v in readings)))


def _check(label, got, want) -> int:
    flag = "ok  " if got == want else "FAIL"
    print(f"{flag} {got:4.2f}  {label}")
    return flag == "FAIL"


def _release_cases(start) -> int:
    bad = 0
    full = {**start, **SEATED_CASE}
    _set_test(THROUGH)
    for label, gaps, want in (("gripper 4.9 mm from the rod", {"pivot_rod": 0.0049, "stopper": 0.05}, 0.0),
                              ("gripper 5.0 mm from the rod", {"pivot_rod": 0.005, "stopper": 0.05}, 1.0),
                              ("gripper still on the stopper's head", {"pivot_rod": 0.05, "stopper": 0.0}, 0.0)):
        task._GAP.clear()
        task._GAP.update(gaps)
        bad += _check(label, 1.0 if task.success(full) else float(task.score(full)), want)
    task._GAP.clear()
    task._TEST.clear()
    os.environ[task.OFFLINE_FLAG] = "0"
    bad += _check("no gripper measurement yet: fails closed", float(task.score(full)), 0.0)
    os.environ[task.OFFLINE_FLAG] = "1"
    return bad


class _FakeSim:
    t = 0.0


def _test_cases(start) -> int:
    """Steps the test's state machine offline. The pushes are not applied (no simulator); the stopper's height is
    scripted, step by step, to what the readings in THROUGH, BESIDE or UNDER would show."""
    bad = 0
    os.environ[task.OFFLINE_FLAG] = "0"  # in the simulator: the release and the test are measured
    real_push = task._push
    task._push = lambda *a: None
    far = {"pivot_rod": 0.05, "stopper": 0.05}

    def run(wait, heights, base=None, near_at=None):
        """`wait` still steps, then one scripted stopper height (m, from its start) per step; near_at: a step on which a
        gripper comes within 1 mm of the rod. Returns the score after the last step."""
        task._new_test()
        base = base or {**start, **SEATED_CASE}
        z0 = float(base["stopper"].pos[2])
        st = base
        for i, dz in enumerate([0.0] * wait + list(heights)):
            task._GAP.clear()
            task._GAP.update({"pivot_rod": 0.001, "stopper": 0.05} if i == near_at else far)
            p = base["stopper"].pos
            st = {**base, "stopper": replace(base["stopper"], pos=(p[0], p[1], z0 + dz))}
            task._advance(_FakeSim(), st)
        return 1.0 if task.success(st) else float(task.score(st))

    d, b, u = DROP_MM / 1000.0, BACK_MM / 1000.0, RISE_MM / 1000.0
    n = task.HOLD_STEPS
    through = [-d] * task.LIFT_STEPS + [-d + b] * (task.BACK_STEPS + task.SETTLE_STEPS) + [-d + b + u] * task.PULL_STEPS
    beside = [0.0] * (task.LIFT_STEPS + task.BACK_STEPS + task.SETTLE_STEPS) + [0.02] * task.PULL_STEPS
    under = [-d] * task.LIFT_STEPS + [-d + b] * (task.BACK_STEPS + task.SETTLE_STEPS) + [-d + b + 0.02] * task.PULL_STEPS
    bad += _check("still only 19 steps: the test has not started", run(n - 1, []), 0.0)
    bad += _check("rod through the eye: test passes", run(n, through), 1.0)
    bad += _check("rod through the eye, ball 3.5 mm out before the test", run(n, through, base={
        **start, **stopper(), **rod(out=0.0035)}), 0.5)
    bad += _check("rod beside the eye: nothing moves, the pull lifts it", run(n, beside), 0.0)
    bad += _check("stopper on a rod under its eye: follows it, lifts off", run(n, under), 0.0)
    bad += _check("gripper near during the lift: test abandoned", run(n, through, near_at=n + 5), 0.0)
    bad += _check("gripper near after the test passed: credit withdrawn", run(n, through + [0.0], near_at=n + len(
        through)), 0.0)
    task._push = real_push
    task._TEST.clear()
    task._GAP.clear()
    os.environ[task.OFFLINE_FLAG] = "1"
    return bad


def _run() -> int:
    start = {o.name: o.state() for o in task.OBJECTS}
    bad = 0
    task._TEST.clear()
    for label, changes, want, want_success in CASES:
        st = {**start, **changes}
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    full = {**start, **SEATED_CASE}
    for label, readings, want in READINGS:
        _set_test(readings)
        bad += _check(label, 1.0 if task.success(full) else float(task.score(full)), want)
    return bad + _release_cases(start) + _test_cases(start)


if __name__ == "__main__":
    raise SystemExit(_run())
