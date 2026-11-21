"""Offline: are the demo's waypoints reachable within joint limits? IK error per segment (no simulator)."""
import numpy as np
from yam_arena import ik, station, task_file
from yam_arena.demo import ArmPlan
T = task_file.load("task.py").module
for arm, wps in T.demo().items():
    plan = ArmPlan(wps, station.BASES[arm]); lim = 0.0; t = 0.0; per = {}
    while t < plan.duration:
        before = plan.worst_ik_mm; plan.worst_ik_mm = 0.0
        q7, label = plan.action7(t); q = np.asarray(q7[:6]); t += 0.05
        per[label] = max(per.get(label, 0.0), plan.worst_ik_mm); plan.worst_ik_mm = max(before, plan.worst_ik_mm)
        lim = max(lim, float(np.max(np.maximum(np.asarray(ik.LOWER) - q, q - np.asarray(ik.UPPER)))))
    print(f"{arm}: {plan.duration:.1f} s ({int(plan.duration*20)} steps), worst IK error {plan.worst_ik_mm:.1f} mm, "
          f"{plan.worst_ik_deg:.1f} deg, joint-limit overshoot {lim:.3f} rad")
    for w in wps:
        print(f"   {w.label:42s} {per.get(w.label, 0.0):5.1f} mm")
