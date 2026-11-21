"""Offline: are the demo's waypoints reachable within the joint limits, and how closely does IK track them?
(no simulator)  Run: python check_demo.py"""
import numpy as np
from yam_arena import ik, station, task_file
from yam_arena.demo import ArmPlan

T = task_file.load("task.py").module
for arm, wps in T.demo().items():
    plan = ArmPlan(wps, station.BASES[arm])
    lim, t = 0.0, 0.0
    while t < plan.duration:
        q7, _ = plan.action7(t)
        q = np.asarray(q7[:6])
        lim = max(lim, float(np.max(np.maximum(np.asarray(ik.LOWER) - q, q - np.asarray(ik.UPPER)))))
        t += 0.05
    print(f"{arm}: {plan.duration:.1f} s ({int(plan.duration * 20)} steps), worst IK error {plan.worst_ik_mm:.1f} mm "
          f"/ {plan.worst_ik_deg:.1f} deg, joint-limit overshoot {lim:.3f} rad")
    for w in wps:
        d = np.linalg.norm(np.asarray(w.pos[:2]) - np.asarray(station.BASES[arm][:2]))
        print(f"   {w.label:32s} xy ({w.pos[0]:.3f}, {w.pos[1]:.3f}) z {w.pos[2] - T.TOP:+.3f}  {d:.3f} m from the base")
