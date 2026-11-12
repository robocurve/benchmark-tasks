from __future__ import annotations
import pathlib, sys, numpy as np
_TASK_DIR = pathlib.Path(__file__).resolve().parent
_UPSTREAM = _TASK_DIR.parents[2] / "robocurve-windows-transfer-2026-11-05" / "project" / "robocurve" / "tmp" / "yam-upstream"
for p in (_TASK_DIR, _UPSTREAM):
    if str(p) not in sys.path: sys.path.insert(0, str(p))
from demo_plan import plans
from yam_arena import ik
from yam_arena.constants import LEFT_BASE_POS, READY, RIGHT_BASE_POS
from yam_arena.demo import grip_rotation

def check_plan(name, waypoints, base_pos, q_init=READY):
    worst_pe_mm = 0.0
    worst_re_deg = 0.0
    min_margin = float("inf")
    q = np.array(q_init, dtype=float)
    for i, w in enumerate(waypoints):
        target = np.asarray(w.pos, dtype=float)
        rot = grip_rotation(w.yaw)
        quat = ik.matrix_to_quat_xyzw(rot)
        q, pe, re = ik.solve(target, quat, q, base_pos=base_pos, iters=100)
        pe_mm = pe * 1000.0
        re_deg = float(np.degrees(re))
        worst_pe_mm = max(worst_pe_mm, pe_mm)
        worst_re_deg = max(worst_re_deg, re_deg)
        margin = np.minimum(q - ik.LOWER, ik.UPPER - q).min()
        min_margin = min(min_margin, float(margin))
        assert pe_mm < 1.0, f"Waypoint {i} ({w.label}) pe too large: {pe_mm:.3f} mm"
        assert re_deg < 5.0, f"Waypoint {i} ({w.label}) re too large: {re_deg:.2f} deg"
        assert margin >= 0.02, f"Waypoint {i} ({w.label}) margin low: {margin:.4f} rad"
    return worst_pe_mm, worst_re_deg, min_margin

def main():
    p = plans()
    print("Checking IK reach for keyboard_tray_latch...")
    l_pe, l_re, l_m = check_plan("left", p["left"], LEFT_BASE_POS)
    r_pe, r_re, r_m = check_plan("right", p["right"], RIGHT_BASE_POS)
    print(f"Left: pe={l_pe:.3f} mm, margin={l_m:.4f} rad | Right: pe={r_pe:.3f} mm, margin={r_m:.4f} rad")
    print("SUCCESS: all waypoints reachable.")

if __name__ == "__main__":
    main()
