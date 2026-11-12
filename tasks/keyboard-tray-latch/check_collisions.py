from __future__ import annotations
import pathlib, sys, numpy as np
_TASK_DIR = pathlib.Path(__file__).resolve().parent
_UPSTREAM = _TASK_DIR.parents[2] / "robocurve-windows-transfer-2026-11-05" / "project" / "robocurve" / "tmp" / "yam-upstream"
for p in (_TASK_DIR, _UPSTREAM):
    if str(p) not in sys.path: sys.path.insert(0, str(p))
from demo_plan import plans
from yam_arena.constants import LEFT_BASE_POS, RIGHT_BASE_POS, TABLE_HEIGHT
from yam_arena.demo import ArmPlan

GRIPPER_WIDTH_M = 0.085

def main():
    p = plans()
    l_plan = ArmPlan(p["left"], LEFT_BASE_POS, finger_start=p["left"][0].finger)
    r_plan = ArmPlan(p["right"], RIGHT_BASE_POS, finger_start=p["right"][0].finger)
    duration = max(l_plan.duration, r_plan.duration)
    dt = 0.05
    steps = int(np.ceil(duration / dt))
    min_tcp = float("inf")
    min_clr = float("inf")
    min_tab = float("inf")
    print(f"Checking clearance over {steps} steps ({duration:.1f} s)...")
    for s in range(steps):
        t = s * dt
        l_pos, _, _, _ = l_plan.target(t)
        r_pos, _, _, _ = r_plan.target(t)
        tcp_dist = float(np.linalg.norm(l_pos - r_pos))
        min_tcp = min(min_tcp, tcp_dist)
        clr = tcp_dist - GRIPPER_WIDTH_M
        min_clr = min(min_clr, clr)
        min_tab = min(min_tab, l_pos[2] - TABLE_HEIGHT, r_pos[2] - TABLE_HEIGHT)
        if tcp_dist < 0.115:
            raise RuntimeError(f"Arm collision at t={t:.2f}s: tcp {tcp_dist*1000:.1f} mm < 115 mm")
        if min_tab < 0.010:
            raise RuntimeError(f"Table collision at t={t:.2f}s")
    print(f"Min TCP: {min_tcp*1000:.1f} mm | Min gripper clearance: {min_clr*1000:.1f} mm | Min table: {min_tab*1000:.1f} mm")
    print("SUCCESS: all clearances verified.")

if __name__ == "__main__":
    main()
