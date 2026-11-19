"""Retiming shared Cartesian motion to respect the joint-command delta guard."""

import math

import fastik
import numpy as np
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.demo import Waypoint


def retime(plans, dt=0.05, joint_delta=0.025):
    """Use a shared duration for both hands; preserve every Cartesian target.

    Cubic interpolation peaks at 1.5 times its average speed. A 0.025 rad
    increment leaves headroom below the 0.05 rad guard for interpolation and
    proprioceptive feedback. Gripper closure is similarly rate limited.
    """
    q = {arm: np.array(READY, dtype=float) for arm in plans}
    finger = {arm: FINGER_OPEN for arm in plans}
    out = {arm: [] for arm in plans}
    end = max(plan.duration for plan in plans.values())
    worst_mm = 0.0
    for t in np.arange(dt, end + dt / 2, dt):
        goals = {arm: plan.target(float(t)) for arm, plan in plans.items()}
        steps = 1
        for arm, (pos, rotation, grip, label) in goals.items():
            new, pe, re = fastik.solve(
                pos, rotation, q[arm], plans[arm].base_pos, iters=80
            )
            worst_mm = max(worst_mm, pe * 1000)
            if pe > 0.001 or re > 0.01:
                raise ValueError(
                    f"Unreachable {arm} pose at {t:.2f}s ({label}): {pe * 1000:.2f} mm"
                )
            steps = max(
                steps,
                math.ceil(float(np.abs(new - q[arm]).max()) / joint_delta),
                math.ceil(abs(grip - finger[arm]) / abs(FINGER_OPEN) / 0.030),
            )
            q[arm], finger[arm] = new, grip
        for arm, (pos, rotation, grip, label) in goals.items():
            out[arm].append(Waypoint(steps * dt, tuple(pos), grip, label, rotation))
    duration = sum(w.duration for w in next(iter(out.values())))
    print(
        f"Retimed bimanual plan: {end:.2f} -> {duration:.2f} s; worst IK {worst_mm:.3f} mm",
        flush=True,
    )
    return out
