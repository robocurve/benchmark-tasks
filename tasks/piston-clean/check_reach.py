"""Sprawdza osiągalność waypointów demo (IK yam_arena, bez symulatora): najgorszy błąd pozycji na etykietę.

Uruchomienie (w środowisku Areny): python check_reach.py piston_clean.py
"""

import sys
from collections import OrderedDict

from yam_arena import demo as yd
from yam_arena import task_file

task = task_file.load(sys.argv[1])
plans = task.demo()
for arm, base in (("left", yd.LEFT_BASE_POS), ("right", yd.RIGHT_BASE_POS)):
    plan = yd.ArmPlan(plans[arm], base)
    worst: "OrderedDict[str, float]" = OrderedDict()
    t, dt = 0.0, 1 / 20
    while t < plan.duration:
        before = plan.worst_ik_mm
        plan.worst_ik_mm = 0.0
        _, label = plan.action7(t)
        worst[label] = max(worst.get(label, 0.0), plan.worst_ik_mm)
        plan.worst_ik_mm = max(before, plan.worst_ik_mm)
        t += dt
    bad = [(k, v) for k, v in worst.items() if v > 5.0]
    print(f"{arm}: worst {plan.worst_ik_mm:.1f} mm over {plan.duration:.1f} s; >5 mm: {len(bad)}")
    for k, v in bad[:20]:
        print(f"   {v:7.1f} mm  {k}")
