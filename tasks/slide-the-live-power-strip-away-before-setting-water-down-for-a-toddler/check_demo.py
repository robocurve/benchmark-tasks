"""Offline: are the demo's waypoints reachable within joint limits? (no simulator)"""
import numpy as np
from yam_arena import ik, station, task_file
from yam_arena.demo import ArmPlan
T = task_file.load("task.py").module
for arm, wps in T.demo().items():
    plan = ArmPlan(wps, station.BASES[arm]); worst = 0; lim = 0; t = 0.0; lab = ""
    while t < plan.duration:
        q7, _ = plan.action7(t); q = np.asarray(q7[:6]); t += 0.05
        lim = max(lim, float(np.max(np.maximum(np.asarray(ik.LOWER) - q, q - np.asarray(ik.UPPER)))))
    worst, lab = plan.worst_ik_mm, f"{plan.worst_ik_deg:.1f} deg"
    print(f"{arm}: {plan.duration:.1f} s ({int(plan.duration*20)} steps), worst IK error {worst:.1f} mm at '{lab}', joint-limit overshoot {lim:.3f} rad")
