"""Check the cage door's lug and the crate door's handle with the harness's IK, without the simulator: straight-down, from the ready pose, along the real paths.

Prints, for each path: the worst IK error, the least margin to a joint limit, and how far forward and how low each link's origin goes. It checks IK only, not collisions.

Usage, with the harness's Python: python check_reach.py
"""

from __future__ import annotations

import numpy as np
from yam_arena import YAM_URDF, ik
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN
from yam_arena.demo import ArmPlan, Waypoint
from yam_arena.station import BASES, TABLE_TOP
from yam_arena.urdf import forward_kinematics


def run(arm: str, label: str, points: list[tuple[float, float, float]], yaw: float = 0.0) -> None:
    base = np.asarray(BASES[arm])
    plan = ArmPlan([Waypoint(2.0, (x, y, TABLE_TOP + z), FINGER_OPEN, "go", yaw) for x, y, z in points], base)
    margin = 9.0
    links: dict[str, list[float]] = {n: [-9.0, 9.0] for n in ("link4", "link5", "gripper")}
    t = 0.0
    while t < plan.duration:
        t += 0.05
        action, _ = plan.action7(t)
        q = action[:6]
        margin = min(margin, float(np.min(np.minimum(q - ik.LOWER, ik.UPPER - q))))
        fk = forward_kinematics(YAM_URDF, {n: float(v) for n, v in zip(ARM_JOINTS, q)})
        if t > 2.0:  # past the first move away from the ready pose
            for n, lim in links.items():
                p = fk[n][:3, 3] + base
                lim[0] = max(lim[0], float(p[0]))
                lim[1] = min(lim[1], float(p[2] - TABLE_TOP))
    far = ", ".join(f"{n} x<={v[0]:.3f} z>={v[1]:.3f}" for n, v in links.items())
    ok = "ok " if plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 2.0 else "NO "
    print(f"{ok}{arm:5s} {label:44s} ik {plan.worst_ik_mm:6.1f} mm {plan.worst_ik_deg:5.1f} deg, joint margin {margin:5.2f} rad; {far}", flush=True)


def check_demo() -> None:
    """Play the task's plan through the same IK: the worst error and the largest joint step per control step, per waypoint."""
    import pathlib
    import sys

    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    import task

    for arm, waypoints in task.demo().items():
        plan = ArmPlan(waypoints, np.asarray(BASES[arm]))
        t, last, prev, marks = 0.0, None, None, {}
        while t < plan.duration:
            t += 0.05
            plan.worst_ik_mm, plan.worst_ik_deg = 0.0, 0.0
            action, label = plan.action7(t)
            q = action[:6]
            step = 0.0 if prev is None else float(np.abs(q - prev).max())
            prev = q
            m = marks.setdefault(label, [0.0, 0.0, 0.0])
            m[0], m[1], m[2] = max(m[0], plan.worst_ik_mm), max(m[1], plan.worst_ik_deg), max(m[2], step)
        print(f"{arm}: {plan.duration:.1f} s")
        for label, (mm, deg, step) in marks.items():
            flag = "" if mm < 2.0 and deg < 2.0 and step < 0.1 else "  <-- check"
            print(f"  {label:40s} ik {mm:5.1f} mm {deg:4.1f} deg, step {step:.3f} rad{flag}")


def main() -> None:
    print("Heights are the TCP above the table.")
    check_demo()
    print("Set paths are in the plan above.")


if __name__ == "__main__":
    main()
