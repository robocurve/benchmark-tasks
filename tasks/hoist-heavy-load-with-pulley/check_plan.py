"""Offline IK check of the scripted demo at 20 Hz (no simulator)."""

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from yam_arena import ik, station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import load  # noqa: E402


def main():
    task = load(pathlib.Path(__file__).with_name("task.py"))
    hz = 20.0
    ok = True
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        worst = {}
        for k in range(round(plan.duration * hz) + 5):
            before = (plan.worst_ik_mm, plan.worst_ik_deg)
            q, label = plan.action7((k + 1) / hz)
            if (plan.worst_ik_mm, plan.worst_ik_deg) != before:
                worst[label] = (plan.worst_ik_mm, plan.worst_ik_deg)
            if np.any(q[:6] < ik.LOWER - 1e-6) or np.any(q[:6] > ik.UPPER + 1e-6):
                print(f"{arm}: joint limit at {label}")
                ok = False
        good = plan.worst_ik_mm < 1.0 and plan.worst_ik_deg < 0.5
        ok &= good
        print(
            f"{arm}: {plan.duration:.1f} s, worst IK {plan.worst_ik_mm:.2f} mm / {plan.worst_ik_deg:.2f} deg"
            f" {'ok' if good else 'FAIL'}"
        )
        if not good:
            for label, (mm, deg) in worst.items():
                print(f"    {label}: {mm:.1f} mm {deg:.1f} deg")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
